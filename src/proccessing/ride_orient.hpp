/**
 * @file ride_orient.hpp
 * @brief Wire @c load_ride() to @c Processor::orient_ride() for offline .sfdat files.
 */

#ifndef RIDE_ORIENT_HPP
#define RIDE_ORIENT_HPP

#include "proccessing/processor.hpp"
#include "pipeline/file_sink.hpp"

#include <string>

namespace sf::proc {

/// Nominal fin high-rate IMU sample rate (Hz).
constexpr double kNominalImuRateHz = 55.0;

/**
 * @brief Load a .sfdat ride and run AHRS / DMP orientation into world frame.
 *
 * @param path       Path to a ride file from @c sf::pipeline::load_ride().
 * @param processor  Pipeline instance (Madgwick config from @c processor).
 * @return Time-sorted @c OrientedSample vector with @c accel_global set.
 */
[[nodiscard]] OrientedRide load_and_orient_ride(
    const std::string& path, Processor& processor);

/**
 * @brief Median sample rate (Hz) from @c elapsed_time_ms spacing.
 * @return 0 if fewer than two samples or no positive deltas.
 */
[[nodiscard]] double estimate_sample_rate_hz(const OrientedRide& ride);

} // namespace sf::proc

#endif // RIDE_ORIENT_HPP
