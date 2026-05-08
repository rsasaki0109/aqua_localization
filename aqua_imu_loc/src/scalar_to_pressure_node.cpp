#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

namespace aqua_imu_loc
{

class ScalarToPressureNode : public rclcpp::Node
{
public:
  explicit ScalarToPressureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("scalar_to_pressure", options)
  {
    load_parameters();

    if (input_type_ == "float32") {
      scalar32_sub_ = create_subscription<std_msgs::msg::Float32>(
        scalar_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) { on_scalar(msg->data); });
    } else {
      scalar64_sub_ = create_subscription<std_msgs::msg::Float64>(
        scalar_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_scalar(msg->data); });
    }

    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      pressure_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "scalar_to_pressure started: scalar=%s type=%s mode=%s pressure=%s frame=%s",
      scalar_topic_.c_str(), input_type_.c_str(), mode_.c_str(), pressure_topic_.c_str(),
      frame_id_.c_str());
  }

private:
  void load_parameters()
  {
    scalar_topic_ = declare_parameter<std::string>("topics.scalar", "/scalar_pressure");
    pressure_topic_ = declare_parameter<std::string>("topics.pressure", "/pressure");
    frame_id_ = declare_parameter<std::string>("frame_id", "pressure_link");
    input_type_ = declare_parameter<std::string>("input_type", "float64");
    mode_ = declare_parameter<std::string>("mode", "pressure_pa");

    reference_pressure_pa_ = declare_parameter<double>("reference_pressure_pa", 101325.0);
    pressure_offset_pa_ = declare_parameter<double>("pressure_offset_pa", 0.0);
    pressure_variance_pa2_ = declare_parameter<double>("pressure_variance_pa2", 0.0);
    water_density_kg_m3_ = declare_parameter<double>("water_density_kg_m3", 1025.0);
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
    depth_offset_m_ = declare_parameter<double>("depth_offset_m", 0.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", -1.0);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10000.0);
    barometer_pressure_offset_ = declare_parameter<double>("barometer_pressure_offset", 0.0);
    barometer_pressure_scale_ = declare_parameter<double>("barometer_pressure_scale", 1.0);

    if (input_type_ != "float64" && input_type_ != "float32") {
      RCLCPP_WARN(get_logger(), "Invalid input_type '%s'; using float64.", input_type_.c_str());
      input_type_ = "float64";
    }
    if (mode_ != "pressure_pa" && mode_ != "depth_m" && mode_ != "ntnu_barometer") {
      RCLCPP_WARN(get_logger(), "Invalid mode '%s'; using pressure_pa.", mode_.c_str());
      mode_ = "pressure_pa";
    }
    if (!std::isfinite(reference_pressure_pa_)) {
      RCLCPP_WARN(get_logger(), "Invalid reference_pressure_pa; using 101325.0.");
      reference_pressure_pa_ = 101325.0;
    }
    if (!std::isfinite(pressure_offset_pa_)) {
      RCLCPP_WARN(get_logger(), "Invalid pressure_offset_pa; using 0.0.");
      pressure_offset_pa_ = 0.0;
    }
    if (pressure_variance_pa2_ < 0.0 || !std::isfinite(pressure_variance_pa2_)) {
      RCLCPP_WARN(get_logger(), "Invalid pressure_variance_pa2; using 0.0.");
      pressure_variance_pa2_ = 0.0;
    }
    if (water_density_kg_m3_ <= 0.0 || !std::isfinite(water_density_kg_m3_)) {
      RCLCPP_WARN(get_logger(), "Invalid water_density_kg_m3; using seawater default 1025.0.");
      water_density_kg_m3_ = 1025.0;
    }
    if (gravity_mps2_ <= 0.0 || !std::isfinite(gravity_mps2_)) {
      RCLCPP_WARN(get_logger(), "Invalid gravity_mps2; using standard gravity 9.80665.");
      gravity_mps2_ = 9.80665;
    }
    if (std::abs(barometer_pressure_scale_) <= 1.0e-12 ||
      !std::isfinite(barometer_pressure_scale_))
    {
      RCLCPP_WARN(get_logger(), "Invalid barometer_pressure_scale; using 1.0.");
      barometer_pressure_scale_ = 1.0;
    }
  }

  void on_scalar(double scalar)
  {
    if (!std::isfinite(scalar)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite scalar sample.");
      return;
    }

    const auto pressure_pa = convert_to_pressure(scalar);
    if (!std::isfinite(pressure_pa)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected invalid pressure output.");
      return;
    }

    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = now();
    pressure.header.frame_id = frame_id_;
    pressure.fluid_pressure = pressure_pa;
    pressure.variance = pressure_variance_pa2_;
    pressure_pub_->publish(pressure);
  }

  double convert_to_pressure(double scalar)
  {
    if (mode_ == "pressure_pa") {
      return scalar + pressure_offset_pa_;
    }

    double depth_m = scalar;
    if (mode_ == "ntnu_barometer") {
      depth_m = -((scalar - barometer_pressure_offset_) / barometer_pressure_scale_);
    }

    const double corrected_depth_m = depth_m + depth_offset_m_;
    if (!std::isfinite(corrected_depth_m)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (corrected_depth_m < min_depth_m_ || corrected_depth_m > max_depth_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected depth %.3f m outside range [%.3f, %.3f] m.",
        corrected_depth_m, min_depth_m_, max_depth_m_);
      return std::numeric_limits<double>::quiet_NaN();
    }

    return reference_pressure_pa_ + water_density_kg_m3_ * gravity_mps2_ * corrected_depth_m;
  }

  std::string scalar_topic_;
  std::string pressure_topic_;
  std::string frame_id_;
  std::string input_type_;
  std::string mode_;

  double reference_pressure_pa_{101325.0};
  double pressure_offset_pa_{0.0};
  double pressure_variance_pa2_{0.0};
  double water_density_kg_m3_{1025.0};
  double gravity_mps2_{9.80665};
  double depth_offset_m_{0.0};
  double min_depth_m_{-1.0};
  double max_depth_m_{10000.0};
  double barometer_pressure_offset_{0.0};
  double barometer_pressure_scale_{1.0};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scalar64_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scalar32_sub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
};

}  // namespace aqua_imu_loc

#ifndef AQUA_IMU_LOC_SCALAR_TO_PRESSURE_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_imu_loc::ScalarToPressureNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
