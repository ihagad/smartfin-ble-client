/**
 * @file sensorlog_parser.hpp
 * @brief Load SensorLog GPS CSV exports into @c GPSReading tracks.
 */

#ifndef SENSORLOG_PARSER_HPP
#define SENSORLOG_PARSER_HPP

#include "gps/gps_types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sf::gps {

/**
 * @brief Options when loading a SensorLog CSV.
 */
struct SensorLogLoadOptions {
    /// If set, @c GPSReading::t_elapsed_s = unix_s - session_t0_unix.
    std::optional<double> session_t0_unix;
    /// If set, fill @c position_enu_m on each fix.
    std::optional<GeoOrigin> enu_origin;
};

/**
 * @brief Parse a SensorLog GPS CSV (fake fixtures and real exports).
 *
 * Requires the location column block documented in docs/SENSORLOG_CSV.md.
 * Extra columns (e.g. loggingTime) are ignored when present.
 *
 * @param path  Path to the CSV file.
 * @param opts  Optional session T0 and ENU origin.
 * @return Time-ordered GPS fixes (one per data row).
 * @throws std::runtime_error on I/O failure, missing columns, or parse errors.
 */
[[nodiscard]] std::vector<GPSReading> load_sensorlog_csv(
    const std::string& path,
    const SensorLogLoadOptions& opts = {});

} // namespace sf::gps

#endif // SENSORLOG_PARSER_HPP
