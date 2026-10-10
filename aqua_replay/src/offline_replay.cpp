// SPDX-License-Identifier: Apache-2.0
// Deterministic offline replay harness.
//
// Loads the stack's nodes as rclcpp components into one process, then feeds
// a rosbag2 to them message by message in measurement-stamp order. After each
// message the single-threaded executor runs until no work is left, so every
// callback chain (including sonar -> UKF feedback) completes before the next
// input arrives. With intra-process delivery, simulated /clock stepping, and
// DDS discovery disabled, the outputs are a pure function of the bag, the
// scenario, and the code: there is no wall-clock pacing, no dropped
// messages, and no cross-talk from stray nodes.
//
// Usage:
//   ros2 run aqua_replay offline_replay --scenario S.yaml [--bag IN] [--out OUT]
//     [--start-offset SEC] [--duration SEC] [--param NODE:NAME:=VALUE]...
//     [--summary PATH] [--quiet]
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "yaml-cpp/yaml.h"

#include "aqua_replay/message_registry.hpp"
#include "aqua_replay/reorder_buffer.hpp"

namespace fs = std::filesystem;

namespace aqua_replay
{
namespace
{

struct NodeSpec
{
  std::string plugin;
  std::string name;
  std::vector<std::string> params_files;
  std::vector<std::pair<std::string, std::string>> parameters;
  std::vector<std::pair<std::string, std::string>> remaps;
};

struct OutputTopic
{
  std::string topic;
  std::string type;
};

struct Scenario
{
  std::string bag;
  std::string input_storage_id;
  std::vector<std::string> input_topics;
  std::map<std::string, std::string> input_remap;
  double start_offset_s{0.0};
  double duration_s{0.0};
  bool header_order{true};
  double reorder_window_s{0.5};
  std::vector<NodeSpec> nodes;
  std::string out_bag;
  std::string out_storage_id{"mcap"};
  std::vector<OutputTopic> out_topics;
};

struct Options
{
  std::string scenario_path;
  std::optional<std::string> bag;
  std::optional<std::string> out;
  std::optional<double> start_offset_s;
  std::optional<double> duration_s;
  std::vector<std::string> param_overrides;  // NODE:NAME:=VALUE
  std::optional<std::string> summary;
  bool quiet{false};
};

[[noreturn]] void fail(const std::string & message)
{
  throw std::runtime_error(message);
}

std::string resolve_path(const std::string & raw, const fs::path & base_dir)
{
  static const std::regex find_pkg(R"(\$\(find-pkg-share\s+([A-Za-z0-9_]+)\))");
  std::string value = raw;
  std::smatch match;
  while (std::regex_search(value, match, find_pkg)) {
    const std::string share = ament_index_cpp::get_package_share_directory(match[1].str());
    value = match.prefix().str() + share + match.suffix().str();
  }
  fs::path path(value);
  if (path.is_relative()) {
    path = base_dir / path;
  }
  return path.lexically_normal().string();
}

std::string yaml_value_string(const YAML::Node & node)
{
  if (node.IsNull()) {
    return "''";
  }
  if (node.IsScalar()) {
    // Quoted YAML scalars (tag "!") and empty strings must stay strings when
    // rcl re-parses "-p name:=value" as YAML.
    if (node.Tag() == "!" || node.Scalar().empty()) {
      std::string quoted = "'";
      for (const char c : node.Scalar()) {
        quoted += c;
        if (c == '\'') {
          quoted += '\'';
        }
      }
      return quoted + "'";
    }
    return node.Scalar();
  }
  YAML::Emitter emitter;
  emitter << YAML::Flow << node;
  return emitter.c_str();
}

void flatten_parameters(
  const YAML::Node & node, const std::string & prefix,
  std::vector<std::pair<std::string, std::string>> & out)
{
  for (const auto & entry : node) {
    const std::string key = prefix.empty() ?
      entry.first.as<std::string>() : prefix + "." + entry.first.as<std::string>();
    if (entry.second.IsMap()) {
      flatten_parameters(entry.second, key, out);
    } else {
      out.emplace_back(key, yaml_value_string(entry.second));
    }
  }
}

Scenario load_scenario(const std::string & path)
{
  const fs::path base_dir = fs::absolute(path).parent_path();
  const YAML::Node root = YAML::LoadFile(path);
  Scenario s;

  const YAML::Node input = root["input"];
  if (input) {
    if (input["bag"]) {s.bag = resolve_path(input["bag"].as<std::string>(), base_dir);}
    if (input["storage_id"]) {s.input_storage_id = input["storage_id"].as<std::string>();}
    if (input["topics"]) {s.input_topics = input["topics"].as<std::vector<std::string>>();}
    if (input["remap"]) {
      for (const auto & e : input["remap"]) {
        s.input_remap[e.first.as<std::string>()] = e.second.as<std::string>();
      }
    }
    if (input["start_offset_s"]) {s.start_offset_s = input["start_offset_s"].as<double>();}
    if (input["duration_s"]) {s.duration_s = input["duration_s"].as<double>();}
    if (input["order"]) {
      const auto order = input["order"].as<std::string>();
      if (order != "header" && order != "receive") {
        fail("input.order must be 'header' or 'receive', got '" + order + "'");
      }
      s.header_order = order == "header";
    }
    if (input["reorder_window_s"]) {s.reorder_window_s = input["reorder_window_s"].as<double>();}
  }

  for (const auto & n : root["nodes"]) {
    NodeSpec spec;
    spec.plugin = n["plugin"].as<std::string>();
    if (n["name"]) {spec.name = n["name"].as<std::string>();}
    if (n["params_files"]) {
      for (const auto & f : n["params_files"]) {
        spec.params_files.push_back(resolve_path(f.as<std::string>(), base_dir));
      }
    }
    if (n["parameters"]) {flatten_parameters(n["parameters"], "", spec.parameters);}
    if (n["remap"]) {
      for (const auto & e : n["remap"]) {
        spec.remaps.emplace_back(e.first.as<std::string>(), e.second.as<std::string>());
      }
    }
    s.nodes.push_back(std::move(spec));
  }

  const YAML::Node output = root["output"];
  if (output) {
    if (output["bag"]) {s.out_bag = resolve_path(output["bag"].as<std::string>(), base_dir);}
    if (output["storage_id"]) {s.out_storage_id = output["storage_id"].as<std::string>();}
    for (const auto & t : output["topics"]) {
      OutputTopic topic;
      if (t.IsScalar()) {
        topic.topic = t.as<std::string>();
      } else {
        topic.topic = t["topic"].as<std::string>();
        if (t["type"]) {topic.type = t["type"].as<std::string>();}
      }
      if (topic.type.empty()) {
        const auto type = default_type_for_topic(topic.topic);
        if (!type) {
          fail("output topic " + topic.topic + " needs an explicit type");
        }
        topic.type = *type;
      }
      s.out_topics.push_back(topic);
    }
  }
  return s;
}

Options parse_options(int argc, char ** argv)
{
  Options o;
  auto need = [&](int & i) -> std::string {
      if (i + 1 >= argc) {
        fail(std::string("missing value for ") + argv[i]);
      }
      return argv[++i];
    };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;
    } else if (arg == "--scenario") {
      o.scenario_path = need(i);
    } else if (arg == "--bag") {
      o.bag = need(i);
    } else if (arg == "--out") {
      o.out = need(i);
    } else if (arg == "--start-offset") {
      o.start_offset_s = std::stod(need(i));
    } else if (arg == "--duration") {
      o.duration_s = std::stod(need(i));
    } else if (arg == "--param") {
      o.param_overrides.push_back(need(i));
    } else if (arg == "--summary") {
      o.summary = need(i);
    } else if (arg == "--quiet") {
      o.quiet = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout <<
        "offline_replay --scenario S.yaml [--bag IN] [--out OUT] [--start-offset SEC]\n"
        "               [--duration SEC] [--param NODE:NAME:=VALUE]... [--summary PATH]\n"
        "               [--quiet]\n";
      std::exit(0);
    } else {
      fail("unknown argument: " + arg);
    }
  }
  if (o.scenario_path.empty()) {
    fail("--scenario is required");
  }
  return o;
}

void apply_param_overrides(Scenario & s, const std::vector<std::string> & overrides)
{
  for (const auto & raw : overrides) {
    // NODE may itself contain "::" (a plugin name), so split at the last ':'
    // before ":=".
    const auto assign = raw.find(":=");
    const auto colon =
      (assign == std::string::npos || assign == 0) ? std::string::npos : raw.rfind(':', assign - 1);
    if (colon == std::string::npos || assign == std::string::npos || colon == 0) {
      fail("--param expects NODE:NAME:=VALUE, got '" + raw + "'");
    }
    const std::string node = raw.substr(0, colon);
    const std::string name = raw.substr(colon + 1, assign - colon - 1);
    const std::string value = raw.substr(assign + 2);
    bool matched = false;
    for (auto & spec : s.nodes) {
      if (spec.name == node || spec.plugin == node) {
        spec.parameters.emplace_back(name, value);
        matched = true;
      }
    }
    if (!matched) {
      fail("--param node '" + node + "' does not match any scenario node name or plugin");
    }
  }
}

void isolate_from_other_ros_processes()
{
  if (const char * allow = std::getenv("AQUA_REPLAY_ALLOW_DISCOVERY"); allow && *allow == '1') {
    return;
  }
#if __has_include(<rmw/discovery_options.h>)
  // Intra-process delivery needs no DDS discovery; turning it off means no
  // leaked node on the same domain can inject messages into the replay.
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "OFF", 1);
#else
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
  const std::string domain = std::to_string(100 + (getpid() % 30));
  setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
#endif
}

template<typename M>
auto receive_ns_impl(const M & m, int) -> decltype(m.recv_timestamp, int64_t())
{
  return static_cast<int64_t>(m.recv_timestamp);
}

template<typename M>
int64_t receive_ns_impl(const M & m, long)
{
  return static_cast<int64_t>(m.time_stamp);
}

int64_t receive_ns(const rosbag2_storage::SerializedBagMessage & m)
{
  return receive_ns_impl(m, 0);
}

struct ComponentLibrary
{
  std::string class_name;
  std::string library_path;
};

std::vector<ComponentLibrary> list_components()
{
  std::vector<ComponentLibrary> components;
  for (const auto & [package, prefix] : ament_index_cpp::get_resources("rclcpp_components")) {
    std::string content;
    std::string base;
    if (!ament_index_cpp::get_resource("rclcpp_components", package, content, &base)) {
      continue;
    }
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
      const auto sep = line.find(';');
      if (sep == std::string::npos) {
        continue;
      }
      ComponentLibrary c;
      c.class_name = line.substr(0, sep);
      fs::path lib(line.substr(sep + 1));
      if (lib.is_relative()) {
        lib = fs::path(base) / lib;
      }
      c.library_path = lib.string();
      components.push_back(c);
    }
  }
  return components;
}

struct LoadedNode
{
  // Declaration order matters: the node wrapper must be destroyed before
  // the loader unloads the library that holds its code.
  std::unique_ptr<class_loader::ClassLoader> loader;
  rclcpp_components::NodeInstanceWrapper wrapper;
  std::string plugin;
};

LoadedNode load_node(const NodeSpec & spec, const std::vector<ComponentLibrary> & components)
{
  const auto it = std::find_if(
    components.begin(), components.end(),
    [&](const ComponentLibrary & c) {return c.class_name == spec.plugin;});
  if (it == components.end()) {
    std::string known;
    for (const auto & c : components) {
      known += "\n  " + c.class_name;
    }
    fail("component plugin '" + spec.plugin + "' not found; available:" + known);
  }

  std::vector<std::string> args{"--ros-args"};
  if (!spec.name.empty()) {
    args.insert(args.end(), {"-r", "__node:=" + spec.name});
  }
  for (const auto & file : spec.params_files) {
    args.insert(args.end(), {"--params-file", file});
  }
  for (const auto & [name, value] : spec.parameters) {
    args.insert(args.end(), {"-p", name + ":=" + value});
  }
  for (const auto & [from, to] : spec.remaps) {
    args.insert(args.end(), {"-r", from + ":=" + to});
  }
  args.insert(args.end(), {"-p", "use_sim_time:=true"});

  rclcpp::NodeOptions options;
  options.use_global_arguments(false)
  .arguments(args)
  .use_intra_process_comms(true)
  .enable_rosout(false)
  .start_parameter_event_publisher(false)
  .use_clock_thread(false);

  LoadedNode loaded;
  loaded.plugin = spec.plugin;
  loaded.loader = std::make_unique<class_loader::ClassLoader>(it->library_path);
  const std::string factory_name = "rclcpp_components::NodeFactoryTemplate<" + spec.plugin + ">";
  std::shared_ptr<rclcpp_components::NodeFactory> factory;
  for (const auto & clazz : loaded.loader->getAvailableClasses<rclcpp_components::NodeFactory>()) {
    if (clazz == spec.plugin || clazz == factory_name) {
      factory = loaded.loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      break;
    }
  }
  if (!factory) {
    fail("library " + it->library_path + " does not export " + spec.plugin);
  }
  loaded.wrapper = factory->create_node_instance(options);
  return loaded;
}

struct InputTopic
{
  std::string bag_topic;
  std::string stack_topic;
  const TypeSupport * support{nullptr};
  rclcpp::PublisherBase::SharedPtr publisher;
  uint64_t delivered{0};
};

struct OutputStats
{
  std::string type;
  uint64_t count{0};
  Fnv1a64 digest;
};

std::string hex64(uint64_t value)
{
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(value));
  return buf;
}

int run(const Options & options)
{
  Scenario scenario = load_scenario(options.scenario_path);
  if (options.bag) {scenario.bag = *options.bag;}
  if (options.out) {scenario.out_bag = *options.out;}
  if (options.start_offset_s) {scenario.start_offset_s = *options.start_offset_s;}
  if (options.duration_s) {scenario.duration_s = *options.duration_s;}
  apply_param_overrides(scenario, options.param_overrides);
  if (scenario.bag.empty()) {fail("no input bag (input.bag or --bag)");}
  if (scenario.out_bag.empty()) {fail("no output bag (output.bag or --out)");}
  if (scenario.nodes.empty()) {fail("scenario has no nodes");}
  if (fs::exists(scenario.out_bag)) {
    fail("output bag already exists: " + scenario.out_bag);
  }

  const auto wall_start = std::chrono::steady_clock::now();

  // --- Input bag --------------------------------------------------------
  rosbag2_cpp::Reader reader;
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = scenario.bag;
    storage.storage_id = scenario.input_storage_id;
    reader.open(storage);
  }
  std::vector<InputTopic> inputs;
  std::map<std::string, size_t> input_index;
  for (const auto & meta : reader.get_all_topics_and_types()) {
    if (!scenario.input_topics.empty() &&
      std::find(scenario.input_topics.begin(), scenario.input_topics.end(), meta.name) ==
      scenario.input_topics.end())
    {
      continue;
    }
    const TypeSupport * support = find_type_support(meta.type);
    if (!support) {
      if (!options.quiet) {
        std::cerr << "[offline_replay] skipping " << meta.name << " (unsupported type "
                  << meta.type << ")\n";
      }
      continue;
    }
    InputTopic in;
    in.bag_topic = meta.name;
    const auto remap = scenario.input_remap.find(meta.name);
    in.stack_topic = remap == scenario.input_remap.end() ? meta.name : remap->second;
    in.support = support;
    input_index[meta.name] = inputs.size();
    inputs.push_back(in);
  }
  for (const auto & wanted : scenario.input_topics) {
    if (!input_index.count(wanted)) {
      fail("input topic " + wanted + " is not in the bag or has an unsupported type");
    }
  }
  if (inputs.empty()) {fail("no replayable input topics");}
  {
    rosbag2_storage::StorageFilter filter;
    for (const auto & in : inputs) {filter.topics.push_back(in.bag_topic);}
    reader.set_filter(filter);
  }
  const int64_t bag_start_ns =
    reader.get_metadata().starting_time.time_since_epoch().count();
  const int64_t start_ns =
    bag_start_ns + static_cast<int64_t>(scenario.start_offset_s * 1e9);
  const int64_t end_ns = scenario.duration_s > 0.0 ?
    start_ns + static_cast<int64_t>(scenario.duration_s * 1e9) :
    std::numeric_limits<int64_t>::max();
  if (scenario.start_offset_s > 0.0) {
    reader.seek(start_ns);
  }

  // --- Nodes ------------------------------------------------------------
  const auto components = list_components();
  std::vector<LoadedNode> nodes;
  for (const auto & spec : scenario.nodes) {
    nodes.push_back(load_node(spec, components));
  }

  rclcpp::NodeOptions harness_options;
  harness_options.use_global_arguments(false)
  .use_intra_process_comms(true)
  .enable_rosout(false)
  .start_parameter_event_publisher(false)
  .use_clock_thread(false);
  auto harness = std::make_shared<rclcpp::Node>("aqua_offline_replay", harness_options);
  auto clock_pub = harness->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(10).reliable());

  for (auto & in : inputs) {
    in.publisher = in.support->create_publisher(
      *harness, in.stack_topic, rclcpp::QoS(1000).reliable());
  }

  // --- Output bag + recorders --------------------------------------------
  rosbag2_cpp::Writer writer;
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = scenario.out_bag;
    storage.storage_id = scenario.out_storage_id;
    writer.open(storage, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  }
  int64_t clock_ns = 0;
  std::map<std::string, OutputStats> output_stats;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> recorders;
  for (const auto & out : scenario.out_topics) {
    const TypeSupport * support = find_type_support(out.type);
    if (!support) {fail("unsupported output type " + out.type + " for " + out.topic);}
    rosbag2_storage::TopicMetadata meta;
    meta.name = out.topic;
    meta.type = out.type;
    meta.serialization_format = "cdr";
    writer.create_topic(meta);
    output_stats[out.topic].type = out.type;
    auto * stats = &output_stats[out.topic];
    const std::string topic = out.topic;
    const std::string type = out.type;
    recorders.push_back(
      support->create_recorder(
        *harness, out.topic, rclcpp::QoS(10000).best_effort(),
        [&writer, &clock_ns, stats, topic, type](std::shared_ptr<rclcpp::SerializedMessage> s) {
          const auto & raw = s->get_rcl_serialized_message();
          stats->digest.update(raw.buffer, raw.buffer_length);
          ++stats->count;
          writer.write(s, topic, type, rclcpp::Time(clock_ns, RCL_ROS_TIME));
        }));
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node.wrapper.get_node_base_interface());
  }
  executor.add_node(harness);
  executor.spin_all(std::chrono::nanoseconds(0));

  // --- Replay -------------------------------------------------------------
  ReorderBuffer buffer;
  std::vector<QueuedMessage> ready;
  uint64_t sequence = 0;
  uint64_t delivered = 0;
  uint64_t late = 0;
  int64_t first_key_ns = -1;
  int64_t last_progress_ns = 0;
  const int64_t window_ns =
    scenario.header_order ? static_cast<int64_t>(scenario.reorder_window_s * 1e9) : 0;

  auto deliver = [&](QueuedMessage & q) {
      if (first_key_ns < 0) {first_key_ns = q.order_key_ns;}
      if (q.order_key_ns > clock_ns) {
        clock_ns = q.order_key_ns;
        rosgraph_msgs::msg::Clock clock;
        clock.clock = rclcpp::Time(clock_ns, RCL_ROS_TIME);
        clock_pub->publish(clock);
        executor.spin_all(std::chrono::nanoseconds(0));
      } else if (q.order_key_ns < clock_ns) {
        ++late;
      }
      InputTopic & in = inputs[q.topic_index];
      in.support->publish(*in.publisher, q.message);
      ++in.delivered;
      ++delivered;
      executor.spin_all(std::chrono::nanoseconds(0));
      if (!options.quiet && clock_ns - last_progress_ns >= 10'000'000'000LL) {
        last_progress_ns = clock_ns;
        std::cerr << "[offline_replay] t=" << (clock_ns - first_key_ns) * 1e-9
                  << " s, delivered " << delivered << "\n";
      }
    };

  while (reader.has_next() && rclcpp::ok()) {
    const auto bag_msg = reader.read_next();
    const int64_t recv_ns = receive_ns(*bag_msg);
    if (recv_ns < start_ns) {continue;}
    if (recv_ns > end_ns) {break;}
    const auto idx = input_index.find(bag_msg->topic_name);
    if (idx == input_index.end()) {continue;}
    InputTopic & in = inputs[idx->second];
    const rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    QueuedMessage q;
    q.message = in.support->deserialize(serialized);
    q.receive_ns = recv_ns;
    q.sequence = sequence++;
    q.topic_index = idx->second;
    q.order_key_ns = recv_ns;
    if (scenario.header_order) {
      if (const auto stamp = in.support->header_stamp_ns(q.message.get()); stamp && *stamp > 0) {
        q.order_key_ns = *stamp;
      }
    }
    buffer.push(std::move(q));
    ready.clear();
    buffer.pop_ready(recv_ns - window_ns, ready);
    for (auto & r : ready) {deliver(r);}
  }
  ready.clear();
  buffer.drain(ready);
  for (auto & r : ready) {deliver(r);}
  executor.spin_all(std::chrono::nanoseconds(0));

  // Tear down in a safe order: stop callbacks, drop recorders, close bag.
  for (auto & node : nodes) {
    executor.remove_node(node.wrapper.get_node_base_interface());
  }
  executor.remove_node(harness);
  recorders.clear();
  writer.close();

  // --- Summary --------------------------------------------------------------
  Fnv1a64 fingerprint;
  YAML::Emitter y;
  y << YAML::BeginMap;
  y << YAML::Key << "scenario" << YAML::Value << fs::absolute(options.scenario_path).string();
  y << YAML::Key << "input_bag" << YAML::Value << scenario.bag;
  y << YAML::Key << "output_bag" << YAML::Value << scenario.out_bag;
  y << YAML::Key << "order" << YAML::Value << (scenario.header_order ? "header" : "receive");
  y << YAML::Key << "reorder_window_s" << YAML::Value << scenario.reorder_window_s;
  y << YAML::Key << "start_offset_s" << YAML::Value << scenario.start_offset_s;
  y << YAML::Key << "duration_s" << YAML::Value << scenario.duration_s;
  y << YAML::Key << "messages_delivered" << YAML::Value << delivered;
  y << YAML::Key << "late_messages" << YAML::Value << late;
  y << YAML::Key << "first_stamp_ns" << YAML::Value << first_key_ns;
  y << YAML::Key << "last_stamp_ns" << YAML::Value << clock_ns;
  y << YAML::Key << "wall_time_s" << YAML::Value <<
    std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  y << YAML::Key << "nodes" << YAML::Value << YAML::BeginSeq;
  for (const auto & spec : scenario.nodes) {
    y << YAML::BeginMap << YAML::Key << "plugin" << YAML::Value << spec.plugin;
    y << YAML::Key << "params_files" << YAML::Value << spec.params_files;
    y << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, value] : spec.parameters) {
      y << YAML::Key << name << YAML::Value << value;
    }
    y << YAML::EndMap << YAML::EndMap;
  }
  y << YAML::EndSeq;
  y << YAML::Key << "inputs" << YAML::Value << YAML::BeginMap;
  for (const auto & in : inputs) {
    y << YAML::Key << in.bag_topic << YAML::Value << YAML::BeginMap;
    y << YAML::Key << "stack_topic" << YAML::Value << in.stack_topic;
    y << YAML::Key << "type" << YAML::Value << in.support->type_name;
    y << YAML::Key << "delivered" << YAML::Value << in.delivered;
    y << YAML::EndMap;
  }
  y << YAML::EndMap;
  y << YAML::Key << "outputs" << YAML::Value << YAML::BeginMap;
  for (const auto & [topic, stats] : output_stats) {
    fingerprint.update(reinterpret_cast<const uint8_t *>(topic.data()), topic.size());
    fingerprint.update_u64(stats.count);
    fingerprint.update_u64(stats.digest.value());
    y << YAML::Key << topic << YAML::Value << YAML::BeginMap;
    y << YAML::Key << "type" << YAML::Value << stats.type;
    y << YAML::Key << "count" << YAML::Value << stats.count;
    y << YAML::Key << "fnv1a64" << YAML::Value << hex64(stats.digest.value());
    y << YAML::EndMap;
  }
  y << YAML::EndMap;
  y << YAML::Key << "fingerprint" << YAML::Value << hex64(fingerprint.value());
  y << YAML::EndMap;

  const std::string summary_path = options.summary ?
    *options.summary : (fs::path(scenario.out_bag) / "replay_summary.yaml").string();
  std::ofstream(summary_path) << y.c_str() << "\n";
  std::cout << "[offline_replay] delivered " << delivered << " messages (" << late
            << " late), fingerprint " << hex64(fingerprint.value()) << "\n"
            << "[offline_replay] summary: " << summary_path << "\n";

  nodes.clear();
  return 0;
}

}  // namespace
}  // namespace aqua_replay

int main(int argc, char ** argv)
{
  try {
    const auto options = aqua_replay::parse_options(argc, argv);
    aqua_replay::isolate_from_other_ros_processes();
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigInt);
    if (options.quiet) {
      rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
    }
    const int rc = aqua_replay::run(options);
    rclcpp::shutdown();
    return rc;
  } catch (const std::exception & e) {
    std::cerr << "[offline_replay] error: " << e.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
