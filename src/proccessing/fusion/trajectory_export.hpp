/**
 * @file trajectory_export.hpp
 * @brief CSV export for fused (and IMU dead-reckoning) trajectories.
 */

#ifndef TRAJECTORY_EXPORT_HPP
#define TRAJECTORY_EXPORT_HPP

#include "fusion/fusion_types.hpp"
#include "gps/gps_types.hpp"
#include "proccessing/proc_types.hpp"

#include <string>
#include <vector>

namespace sf::fusion {

/**
 * @brief Integrate oriented IMU accel only (no GPS updates).
 *
 * Seeds position/velocity from @p seed; uses the same kinematic model as the EKF
 * predict step. Useful for comparing drift vs fused/GPS paths.
 */
[[nodiscard]] std::vector<FusedState> dead_reckon_imu(
    const sf::proc::OrientedRide& imu,
    const gps::GPSReading&        seed,
    const gps::GeoOrigin&         origin,
    double                        max_predict_dt_s = 0.5);

/**
 * @brief Write fused (or DR) states to a SensorLog-style trajectory CSV.
 */
void write_trajectory_csv(
    const std::string&              path,
    const std::vector<FusedState>&  track);

} // namespace sf::fusion

#endif // TRAJECTORY_EXPORT_HPP
