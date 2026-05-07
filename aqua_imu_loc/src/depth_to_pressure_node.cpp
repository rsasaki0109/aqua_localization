#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"

namespace aqua_imu_loc
{

class DepthToPressureNode : public rclcpp::Node
{
public:
  explicit DepthToPressureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("depth_to_pressure", options)
  {
    load_parameters();

    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthToPressureNode::on_depth, this, std::placeholders::_1));
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      pressure_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "depth_to_pressure started: depth=%s pressure=%s frame=%s reference=%.2f density=%.2f gravity=%.5f",
      depth_topic_.c_str(), pressure_topic_.c_str(), frame_id_.c_str(), reference_pressure_pa_,
      water_density_kg_m3_, gravity_mps2_);
  }

private:
  void load_parameters()
  {
    depth_topic_ = declare_parameter<std::string>("topics.depth", "/depth");
    pressure_topic_ = declare_parameter<std::string>("topics.pressure", "/pressure");
    frame_id_ = declare_parameter<std::string>("frame_id", "pressure_link");
    reference_pressure_pa_ = declare_parameter<double>("reference_pressure_pa", 101325.0);
    water_density_kg_m3_ = declare_parameter<double>("water_density_kg_m3", 1025.0);
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
    depth_offset_m_ = declare_parameter<double>("depth_offset_m", 0.0);
    pressure_variance_pa2_ = declare_parameter<double>("pressure_variance_pa2", 0.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", -1.0);

    if (water_density_kg_m3_ <= 0.0 || !std::isfinite(water_density_kg_m3_)) {
      RCLCPP_WARN(get_logger(), "Invalid water_density_kg_m3; using seawater default 1025.0.");
      water_density_kg_m3_ = 1025.0;
    }
    if (gravity_mps2_ <= 0.0 || !std::isfinite(gravity_mps2_)) {
      RCLCPP_WARN(get_logger(), "Invalid gravity_mps2; using standard gravity 9.80665.");
      gravity_mps2_ = 9.80665;
    }
    if (!std::isfinite(reference_pressure_pa_)) {
      RCLCPP_WARN(get_logger(), "Invalid reference_pressure_pa; using 101325.0.");
      reference_pressure_pa_ = 101325.0;
    }
    if (pressure_variance_pa2_ < 0.0 || !std::isfinite(pressure_variance_pa2_)) {
      RCLCPP_WARN(get_logger(), "Invalid pressure_variance_pa2; using 0.0.");
      pressure_variance_pa2_ = 0.0;
    }
  }

  void on_depth(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double corrected_depth_m = msg->data + depth_offset_m_;
    if (!std::isfinite(corrected_depth_m)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite depth sample.");
      return;
    }
    if (corrected_depth_m < min_depth_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected depth %.3f m below min_depth_m %.3f m.", corrected_depth_m, min_depth_m_);
      return;
    }

    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = now();
    pressure.header.frame_id = frame_id_;
    pressure.fluid_pressure =
      reference_pressure_pa_ + water_density_kg_m3_ * gravity_mps2_ * corrected_depth_m;
    pressure.variance = pressure_variance_pa2_;
    pressure_pub_->publish(pressure);
  }

  std::string depth_topic_;
  std::string pressure_topic_;
  std::string frame_id_;
  double reference_pressure_pa_{101325.0};
  double water_density_kg_m3_{1025.0};
  double gravity_mps2_{9.80665};
  double depth_offset_m_{0.0};
  double pressure_variance_pa2_{0.0};
  double min_depth_m_{-1.0};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
};

}  // namespace aqua_imu_loc

#ifndef AQUA_IMU_LOC_DEPTH_TO_PRESSURE_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_imu_loc::DepthToPressureNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
