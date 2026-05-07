#ifndef AQUA_IMU_LOC__PRESSURE_DEPTH_CONVERTER_HPP_
#define AQUA_IMU_LOC__PRESSURE_DEPTH_CONVERTER_HPP_

#include <optional>

namespace aqua_imu_loc
{

struct PressureDepthConfig
{
  bool use_first_pressure_as_reference{true};
  double reference_pressure_pa{101325.0};
  double water_density_kg_m3{1025.0};
  double gravity_mps2{9.80665};
  double depth_offset_m{0.0};
};

class PressureDepthConverter
{
public:
  void configure(const PressureDepthConfig & config);
  std::optional<double> pressure_to_depth(double pressure_pa);

  double reference_pressure_pa() const;
  bool reference_initialized() const;

private:
  PressureDepthConfig config_;
  bool reference_initialized_{false};
};

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__PRESSURE_DEPTH_CONVERTER_HPP_
