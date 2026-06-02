/**
 * @file session_align.hpp
 * @brief Align SensorLog GPS timestamps to fin IMU @c elapsed_time_ms.
 */

#ifndef SESSION_ALIGN_HPP
#define SESSION_ALIGN_HPP

#include "gps/gps_types.hpp"
#include "pipeline/file_sink.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sf::gps {

/// Minimum GPS/IMU timeline overlap (fraction of union span) before warning.
constexpr double kMinSessionOverlapFraction = 0.80;

/**
 * @brief Inclusive elapsed-time span in milliseconds.
 */
struct TimeSpanMs {
    std::uint32_t start_ms = 0;
    std::uint32_t end_ms   = 0;
};

/**
 * @brief Result of aligning GPS fixes to an IMU session clock.
 */
struct SessionAlignmentReport {
    std::vector<GPSReading> gps;
    TimeSpanMs imu_span{};
    TimeSpanMs gps_span{};
    /// Fraction of union timeline covered by both GPS and IMU.
    double overlap_fraction = 0.0;
    std::vector<std::string> warnings;
};

/**
 * @brief Collect every IMU/quat-IMU @c elapsed_time_ms from a loaded ride.
 */
[[nodiscard]] std::vector<std::uint32_t> collect_imu_elapsed_ms(
    const sf::pipeline::RideData& ride);

/**
 * @brief Map GPS Unix time to fin @c elapsed_time_ms using session T0.
 *
 * Sets @c GPSReading::elapsed_time_ms and @c t_elapsed_s on each fix.
 * Computes timeline overlap vs @p imu_elapsed_ms; logs warnings to stderr
 * when overlap &lt; @c kMinSessionOverlapFraction.
 *
 * @param gps             GPS track (sorted by @c unix_s).
 * @param imu_elapsed_ms  Fin IMU timestamps (any order).
 * @param t0_unix_ms      Session start: Unix epoch milliseconds (BLE / SensorLog T0).
 */
[[nodiscard]] SessionAlignmentReport align_sessions(
    std::vector<GPSReading> gps,
    const std::vector<std::uint32_t>& imu_elapsed_ms,
    std::int64_t t0_unix_ms);

/**
 * @brief Inclusive intersection length (ms) of two spans; 0 if disjoint.
 */
[[nodiscard]] std::uint32_t overlap_duration_ms(TimeSpanMs a, TimeSpanMs b);

/**
 * @brief Overlap fraction = intersection / union span (0 if union empty).
 */
[[nodiscard]] double overlap_fraction(TimeSpanMs gps, TimeSpanMs imu);

} // namespace sf::gps

#endif // SESSION_ALIGN_HPP
