/**
 * @file ekf_fusion.hpp
 * @brief Constant-acceleration EKF fusing oriented IMU with SensorLog GPS.
 */

#ifndef EKF_FUSION_HPP
#define EKF_FUSION_HPP

#include "fusion/fusion_types.hpp"
#include "gps/gps_types.hpp"
#include "proccessing/proc_types.hpp"

#include <cstdint>
#include <vector>

namespace sf::fusion {

/**
 * @brief Tunable EKF and coast-mode parameters.
 */
struct EkfFusionConfig {
    /// 1σ acceleration process noise (m/s²) for IMU-driven predict.
    double accel_process_noise_mps2 = 2.0;
    /// Maximum IMU predict step (s); larger gaps are clamped.
    double max_predict_dt_s = 0.5;
    /// When true, zero-acceleration predict with growing P after GPS gaps.
    bool enable_coast = true;
    /// Coast after this many ms without a GPS update.
    std::uint32_t coast_after_gps_gap_ms = 2000;
    /// Multiplier applied to process noise each coast predict step.
    double coast_process_inflation = 2.0;
    /// Initial position 1σ (m) before first GPS update.
    double init_pos_sigma_m = 10.0;
    /// Initial velocity 1σ (m/s).
    double init_vel_sigma_mps = 2.0;
};

/**
 * @brief 6-state EKF: ENU position and velocity.
 *
 * Predict with @c accel_global (g) from oriented IMU; update with GPS ENU
 * position and horizontal velocity from speed/course when valid.
 */
class EkfFusion {
  public:
    explicit EkfFusion(const gps::GeoOrigin& origin,
                       const EkfFusionConfig&   config = {});

    /// Seed state from a GPS fix with @c have_enu.
    void initialize(const gps::GPSReading& fix);

    /// IMU-driven predict; @p accel_global is zero-g acceleration in g (ENU).
    void predict(const math3d::Vec3& accel_global_g, double dt_s, bool coasting);

    /// GPS position (+ optional velocity) measurement update.
    void update(const gps::GPSReading& fix);

    [[nodiscard]] bool initialized() const { return initialized_; }

    [[nodiscard]] FusedState snapshot(std::uint32_t elapsed_time_ms) const;

    [[nodiscard]] std::uint32_t last_gps_update_ms() const
    {
        return last_gps_update_ms_;
    }

  private:
    gps::GeoOrigin   origin_;
    EkfFusionConfig  config_;
    bool             initialized_ = false;
    std::uint32_t    last_gps_update_ms_ = 0;

    double x_[6]{};
    double P_[6][6]{};

    void set_covariance_diagonal(double pos_var, double vel_var);
};

/**
 * @brief Fuse aligned GPS fixes with an oriented IMU ride.
 *
 * @pre @p gps fixes have @c have_enu, @c elapsed_time_ms aligned to IMU clock.
 * @pre @p imu.samples sorted by @c elapsed_time_ms.
 * @return Fused state at each IMU sample time (empty if GPS or IMU empty).
 */
[[nodiscard]] std::vector<FusedState> fuse_gps_imu(
    const sf::proc::OrientedRide&       imu,
    const std::vector<gps::GPSReading>& gps,
    const gps::GeoOrigin&               origin,
    const EkfFusionConfig&              config = {});

} // namespace sf::fusion

#endif // EKF_FUSION_HPP
