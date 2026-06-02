/**
 * @file fusion_types.hpp
 * @brief Output types for GPS–IMU EKF fusion.
 */

#ifndef FUSION_TYPES_HPP
#define FUSION_TYPES_HPP

#include "gps/gps_types.hpp"
#include "proccessing/math/lin_alg.hpp"

#include <cstdint>
#include <vector>

namespace sf::fusion {

/**
 * @brief Position/velocity uncertainty summary (1σ, meters / m/s).
 */
struct FusedUncertainty {
    double pos_horizontal_m = 0.0;
    double pos_vertical_m   = 0.0;
    double vel_mps          = 0.0;
};

/**
 * @brief One fused navigation snapshot after EKF predict/update.
 *
 * Velocity, speed, and heading are the filter state (not raw SensorLog speed).
 */
struct FusedState {
    std::uint32_t elapsed_time_ms = 0;
    double        t_elapsed_s      = 0.0;

    gps::ENU position_enu_m{};
    gps::ENU velocity_enu_mps{};

    double lat_deg     = 0.0;
    double lon_deg     = 0.0;
    double alt_m       = 0.0;
    double speed_mps   = 0.0;
    double heading_deg = -1.0;

    FusedUncertainty uncertainty{};
    /// True when coasting (no recent GPS, constant-velocity predict, inflated P).
    bool coasting = false;
};

} // namespace sf::fusion

#endif // FUSION_TYPES_HPP
