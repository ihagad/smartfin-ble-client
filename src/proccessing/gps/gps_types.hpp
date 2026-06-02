/**
 * @file gps_types.hpp
 * @brief Plain data types for SensorLog GPS loading and ENU conversion.
 */

#ifndef GPS_TYPES_HPP
#define GPS_TYPES_HPP

#include "proccessing/math/lin_alg.hpp"

#include <cstdint>

namespace sf::gps {

/// Local tangent-plane position: x = East, y = North, z = Up (meters).
using ENU = math3d::Vec3;

/**
 * @brief WGS84 anchor for lat/lon ↔ ENU conversion.
 */
struct GeoOrigin {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m   = 0.0;
};

/**
 * @brief One SensorLog GPS fix (Core Location semantics).
 *
 * Negative @c speed_mps, @c course_deg, or accuracy fields mean invalid,
 * matching Apple exports. @c t_elapsed_s and @c position_enu_m are filled
 * when loading with session T0 and/or a @c GeoOrigin.
 */
struct GPSReading {
    double unix_s = 0.0;
    /// Session-relative seconds (fin / SensorLog alignment).
    double t_elapsed_s = 0.0;
    /// Session-relative milliseconds; matches @c elapsed_time_ms on IMU samples.
    std::uint32_t elapsed_time_ms = 0;

    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m   = 0.0;

    double speed_mps              = -1.0;
    double speed_accuracy_mps     = -1.0;
    double course_deg             = -1.0;
    double course_accuracy_deg    = -1.0;
    double vertical_accuracy_m    = -1.0;
    double horizontal_accuracy_m  = -1.0;

    std::int32_t floor_z = 0;

    ENU position_enu_m{};
    bool have_enu = false;

    [[nodiscard]] bool speed_valid() const { return speed_mps >= 0.0; }
    [[nodiscard]] bool course_valid() const { return course_deg >= 0.0; }
};

} // namespace sf::gps

#endif // GPS_TYPES_HPP
