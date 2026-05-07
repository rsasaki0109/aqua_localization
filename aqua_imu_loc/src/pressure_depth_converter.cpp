#include "aqua_imu_loc/pressure_depth_converter.hpp"

#include <cmath>

namespace aqua_imu_loc
{

void PressureDepthConverter::configure(const PressureDepthConfig & config)
{
  config_ = config;
  reference_initialized_ = !config_.use_first_pressure_as_reference;
}

std::optional<double> PressureDepthConverter::pressure_to_depth(double pressure_pa)
{
  if (!std::isfinite(pressure_pa) || config_.water_density_kg_m3 <= 0.0 ||
    config_.gravity_mps2 <= 0.0)
  {
    return std::nullopt;
  }

  if (config_.use_first_pressure_as_reference && !reference_initialized_) {
    config_.reference_pressure_pa = pressure_pa;
    reference_initialized_ = true;
  }

  const double depth_m =
    ((pressure_pa - config_.reference_pressure_pa) /
    (config_.water_density_kg_m3 * config_.gravity_mps2)) + config_.depth_offset_m;
  return depth_m;
}

double PressureDepthConverter::reference_pressure_pa() const
{
  return config_.reference_pressure_pa;
}

bool PressureDepthConverter::reference_initialized() const
{
  return reference_initialized_;
}

}  // namespace aqua_imu_loc
