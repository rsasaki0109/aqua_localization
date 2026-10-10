#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

// Explicit publisher raw-value profile, not an assumption that FluidPressure is SI Pa.
// This matches the upstream node's first-measurement anchoring convention.
namespace aqua_benchmark {
struct PressureEvent {
  uint64_t ordinal;
  int64_t timestamp_ns;
  double raw_value;
  std::string disposition;
};

class ReaqPressureEvents {
 public:
  using Sink = std::function<void(double, int64_t)>;
  ReaqPressureEvents(double raw_offset, double raw_units_per_meter)
      : offset_(raw_offset), scale_(raw_units_per_meter) {
    if (!std::isfinite(offset_) || !std::isfinite(scale_) || scale_ <= 0)
      throw std::invalid_argument("invalid registered pressure profile");
  }

  PressureEvent push(uint64_t ordinal, int64_t stamp, double raw,
                     bool initialized, int64_t imu_first, int64_t imu_last,
                     double estimated_z, const Sink& submit) {
    // A caller that silently drops packets or overwrites duplicate epochs must fail.
    if (ordinal != count_ || (count_ && stamp <= previous_stamp_))
      throw std::invalid_argument("pressure packet ordinal/clock discontinuity");
    if (imu_first > imu_last || !submit)
      throw std::invalid_argument("missing pressure support/update contract");
    PressureEvent event{ordinal, stamp, raw, ""};
    ++count_;
    previous_stamp_ = stamp;
    if (!std::isfinite(raw)) event.disposition = "nonfinite_raw_pressure";
    else if (stamp < imu_first || stamp > imu_last)
      event.disposition = "outside_available_imu_support";
    else if (!initialized) event.disposition = "estimator_not_initialized";
    else if (!std::isfinite(estimated_z))
      throw std::invalid_argument("nonfinite estimator anchor");
    else {
      const double depth = -(raw - offset_) / scale_;
      if (!std::isfinite(depth)) event.disposition = "nonfinite_converted_depth";
      else if (!anchored_) {
        anchor_ = estimated_z - depth;
        if (!std::isfinite(anchor_)) throw std::invalid_argument("nonfinite anchor");
        anchored_ = true;
        event.disposition = "initial_pressure_anchor_no_update";
      } else {
        const double observation = depth + anchor_;
        if (!std::isfinite(observation)) throw std::invalid_argument("nonfinite observation");
        submit(observation, stamp);
        ++updates_;
        event.disposition = "submitted_pressure_update";
      }
    }
    return event;  // Persist every event alongside the unchanged native packet.
  }
  uint64_t native_count() const { return count_; }
  uint64_t submitted_updates() const { return updates_; }

 private:
  double offset_, scale_, anchor_ = 0;
  bool anchored_ = false;
  uint64_t count_ = 0, updates_ = 0;
  int64_t previous_stamp_ = 0;
};
}  // namespace aqua_benchmark
