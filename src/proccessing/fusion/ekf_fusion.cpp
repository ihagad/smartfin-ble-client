/**
 * @file ekf_fusion.cpp
 * @brief Constant-acceleration EKF for GPS–IMU fusion.
 */

#include "fusion/ekf_fusion.hpp"

#include "gps/geo_utils.hpp"
#include "proccessing/math/constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sf::fusion {
namespace {

constexpr int kDim = 6;

constexpr int kPx = 0;
constexpr int kPy = 1;
constexpr int kPz = 2;
constexpr int kVx = 3;
constexpr int kVy = 4;
constexpr int kVz = 5;

using Mat6 = double[6][6];

void mat6_zero(Mat6& m)
{
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            m[r][c] = 0.0;
        }
    }
}

void mat6_identity(Mat6& m)
{
    mat6_zero(m);
    for (int i = 0; i < kDim; ++i) {
        m[i][i] = 1.0;
    }
}

void mat6_copy(const Mat6& src, Mat6& dst)
{
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            dst[r][c] = src[r][c];
        }
    }
}

void mat6_symmetrize(Mat6& p)
{
    for (int r = 0; r < kDim; ++r) {
        for (int c = r + 1; c < kDim; ++c) {
            const double v = 0.5 * (p[r][c] + p[c][r]);
            p[r][c] = v;
            p[c][r] = v;
        }
    }
}

void mat6_add(const Mat6& a, const Mat6& b, Mat6& out)
{
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            out[r][c] = a[r][c] + b[r][c];
        }
    }
}

void mat6_mul(const Mat6& a, const Mat6& b, Mat6& out)
{
    Mat6 tmp{};
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            double sum = 0.0;
            for (int k = 0; k < kDim; ++k) {
                sum += a[r][k] * b[k][c];
            }
            tmp[r][c] = sum;
        }
    }
    mat6_copy(tmp, out);
}

void mat6_transpose(const Mat6& a, Mat6& out)
{
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            out[r][c] = a[c][r];
        }
    }
}

double clamp_positive(double v, double fallback)
{
    return v > 0.0 ? v : fallback;
}

double horizontal_speed(const math3d::Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

double heading_deg_from_velocity(const math3d::Vec3& v)
{
    if (horizontal_speed(v) < 1e-6) {
        return -1.0;
    }
    double deg = std::atan2(v.x, v.y) * RAD_TO_DEG;
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg;
}

math3d::Vec3 velocity_from_gps(const gps::GPSReading& fix)
{
    if (!fix.speed_valid() || !fix.course_valid()) {
        return {};
    }
    const double course_rad = fix.course_deg * DEG_TO_RAD;
    return math3d::Vec3{
        fix.speed_mps * std::sin(course_rad),
        fix.speed_mps * std::cos(course_rad),
        0.0,
    };
}

} // namespace

EkfFusion::EkfFusion(const gps::GeoOrigin& origin, const EkfFusionConfig& config)
    : origin_(origin), config_(config)
{
    set_covariance_diagonal(
        config_.init_pos_sigma_m * config_.init_pos_sigma_m,
        config_.init_vel_sigma_mps * config_.init_vel_sigma_mps);
}

void EkfFusion::set_covariance_diagonal(double pos_var, double vel_var)
{
    mat6_zero(P_);
    P_[kPx][kPx] = pos_var;
    P_[kPy][kPy] = pos_var;
    P_[kPz][kPz] = pos_var;
    P_[kVx][kVx] = vel_var;
    P_[kVy][kVy] = vel_var;
    P_[kVz][kVz] = vel_var;
}

void EkfFusion::initialize(const gps::GPSReading& fix)
{
    if (!fix.have_enu) {
        throw std::invalid_argument("EkfFusion::initialize requires ENU position");
    }

    x_[kPx] = fix.position_enu_m.x;
    x_[kPy] = fix.position_enu_m.y;
    x_[kPz] = fix.position_enu_m.z;

    const math3d::Vec3 v0 = velocity_from_gps(fix);
    x_[kVx]                 = v0.x;
    x_[kVy]                 = v0.y;
    x_[kVz]                 = v0.z;

    set_covariance_diagonal(
        clamp_positive(fix.horizontal_accuracy_m, config_.init_pos_sigma_m)
            * clamp_positive(fix.horizontal_accuracy_m, config_.init_pos_sigma_m),
        clamp_positive(fix.speed_accuracy_mps, config_.init_vel_sigma_mps)
            * clamp_positive(fix.speed_accuracy_mps, config_.init_vel_sigma_mps));

    last_gps_update_ms_ = fix.elapsed_time_ms;
    initialized_        = true;
}

void EkfFusion::predict(const math3d::Vec3& accel_global_g, double dt_s, bool coasting)
{
    if (!initialized_ || dt_s <= 0.0) {
        return;
    }

    dt_s = std::min(dt_s, config_.max_predict_dt_s);

    const math3d::Vec3 accel_mps2 = coasting
        ? math3d::Vec3{}
        : accel_global_g * G0;

    x_[kPx] += x_[kVx] * dt_s + 0.5 * accel_mps2.x * dt_s * dt_s;
    x_[kPy] += x_[kVy] * dt_s + 0.5 * accel_mps2.y * dt_s * dt_s;
    x_[kPz] += x_[kVz] * dt_s + 0.5 * accel_mps2.z * dt_s * dt_s;
    x_[kVx] += accel_mps2.x * dt_s;
    x_[kVy] += accel_mps2.y * dt_s;
    x_[kVz] += accel_mps2.z * dt_s;

    Mat6 F{};
    mat6_identity(F);
    F[kPx][kVx] = dt_s;
    F[kPy][kVy] = dt_s;
    F[kPz][kVz] = dt_s;

    Mat6 FP{};
    mat6_mul(F, P_, FP);
    Mat6 FT{};
    mat6_transpose(F, FT);
    Mat6 FPFt{};
    mat6_mul(FP, FT, FPFt);

    const double sigma_a = config_.accel_process_noise_mps2
        * (coasting ? config_.coast_process_inflation : 1.0);
    const double var_a   = sigma_a * sigma_a;
    const double dt2     = dt_s * dt_s;
    const double q_pp    = 0.25 * dt2 * dt2 * var_a;
    const double q_pv    = 0.5 * dt2 * dt_s * var_a;
    const double q_vv    = dt2 * var_a;

    Mat6 Q{};
    mat6_zero(Q);
    for (int i = 0; i < 3; ++i) {
        const int p = i;
        const int v = i + 3;
        Q[p][p]     = q_pp;
        Q[p][v]     = q_pv;
        Q[v][p]     = q_pv;
        Q[v][v]     = q_vv;
    }

    mat6_add(FPFt, Q, P_);
    mat6_symmetrize(P_);
}

namespace {

void kalman_update(
    double* x, Mat6& P, const double* z, const double* R_diag, int meas_dim,
    const int* state_indices)
{
    // Joseph-form simplified update for diagonal R and row-wise H.
    double y[3]{};
    double S[3][3]{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] = 0.0;
        }
    }

    for (int i = 0; i < meas_dim; ++i) {
        const int idx = state_indices[i];
        y[i]            = z[i] - x[idx];
        S[i][i]         = P[idx][idx] + R_diag[i];
        for (int j = i + 1; j < meas_dim; ++j) {
            const int jdx = state_indices[j];
            S[i][j]       = P[idx][jdx];
            S[j][i]       = S[i][j];
        }
    }

    double S_inv[3][3]{};
    if (meas_dim == 1) {
        S_inv[0][0] = 1.0 / S[0][0];
    } else if (meas_dim == 2) {
        const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        const double inv_det = 1.0 / det;
        S_inv[0][0]          = S[1][1] * inv_det;
        S_inv[0][1]          = -S[0][1] * inv_det;
        S_inv[1][0]          = -S[1][0] * inv_det;
        S_inv[1][1]          = S[0][0] * inv_det;
    } else {
        // 3x3 inverse by cofactors (measurement dim ≤ 3).
        const double det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                         - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                         + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        const double inv_det = 1.0 / det;
        const double c00     = (S[1][1] * S[2][2] - S[1][2] * S[2][1]);
        const double c01     = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]);
        const double c02     = (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        const double c10     = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]);
        const double c11     = (S[0][0] * S[2][2] - S[0][2] * S[2][0]);
        const double c12     = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]);
        const double c20     = (S[0][1] * S[1][2] - S[0][2] * S[1][1]);
        const double c21     = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]);
        const double c22     = (S[0][0] * S[1][1] - S[0][1] * S[1][0]);
        S_inv[0][0]          = c00 * inv_det;
        S_inv[0][1]          = c10 * inv_det;
        S_inv[0][2]          = c20 * inv_det;
        S_inv[1][0]          = c01 * inv_det;
        S_inv[1][1]          = c11 * inv_det;
        S_inv[1][2]          = c21 * inv_det;
        S_inv[2][0]          = c02 * inv_det;
        S_inv[2][1]          = c12 * inv_det;
        S_inv[2][2]          = c22 * inv_det;
    }

    double K[6][3]{};
    for (int c = 0; c < meas_dim; ++c) {
        const int state_idx = state_indices[c];
        for (int r = 0; r < kDim; ++r) {
            double sum = 0.0;
            for (int j = 0; j < meas_dim; ++j) {
                sum += P[r][state_indices[j]] * S_inv[j][c];
            }
            K[r][c] = sum;
        }
    }

    for (int r = 0; r < kDim; ++r) {
        double dx = 0.0;
        for (int i = 0; i < meas_dim; ++i) {
            dx += K[r][i] * y[i];
        }
        x[r] += dx;
    }

    Mat6 I{};
    mat6_identity(I);
    Mat6 KH{};
    mat6_zero(KH);
    for (int r = 0; r < kDim; ++r) {
        for (int i = 0; i < meas_dim; ++i) {
            KH[r][state_indices[i]] = K[r][i];
        }
    }

    Mat6 IKH{};
    for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
            IKH[r][c] = I[r][c] - KH[r][c];
        }
    }

    Mat6 newP{};
    mat6_mul(IKH, P, newP);
    mat6_copy(newP, P);
    mat6_symmetrize(P);
}

} // namespace

void EkfFusion::update(const gps::GPSReading& fix)
{
    if (!fix.have_enu) {
        return;
    }

    if (!initialized_) {
        initialize(fix);
        return;
    }

    const double pos_h_sigma =
        clamp_positive(fix.horizontal_accuracy_m, config_.init_pos_sigma_m);
    const double pos_v_sigma =
        clamp_positive(fix.vertical_accuracy_m, config_.init_pos_sigma_m);

    const double z_pos[3] = {
        fix.position_enu_m.x,
        fix.position_enu_m.y,
        fix.position_enu_m.z,
    };
    const double R_pos[3] = {
        pos_h_sigma * pos_h_sigma,
        pos_h_sigma * pos_h_sigma,
        pos_v_sigma * pos_v_sigma,
    };
    const int pos_idx[3] = {kPx, kPy, kPz};
    kalman_update(x_, P_, z_pos, R_pos, 3, pos_idx);

    const math3d::Vec3 z_vel = velocity_from_gps(fix);
    if (fix.speed_valid() && fix.course_valid()) {
        const double vel_sigma =
            clamp_positive(fix.speed_accuracy_mps, config_.init_vel_sigma_mps);
        const double z_v[2]  = {z_vel.x, z_vel.y};
        const double R_v[2]  = {vel_sigma * vel_sigma, vel_sigma * vel_sigma};
        const int    vel_idx[2] = {kVx, kVy};
        kalman_update(x_, P_, z_v, R_v, 2, vel_idx);
    }

    last_gps_update_ms_ = fix.elapsed_time_ms;
}

FusedState EkfFusion::snapshot(std::uint32_t elapsed_time_ms) const
{
    FusedState out{};
    out.elapsed_time_ms = elapsed_time_ms;
    out.t_elapsed_s     = static_cast<double>(elapsed_time_ms) * MS_TO_S;

    out.position_enu_m = gps::ENU{x_[kPx], x_[kPy], x_[kPz]};
    out.velocity_enu_mps =
        gps::ENU{x_[kVx], x_[kVy], x_[kVz]};

    const auto [lat, lon] = gps::enu_to_latlon(out.position_enu_m, origin_);
    out.lat_deg             = lat;
    out.lon_deg             = lon;
    out.alt_m               = gps::enu_to_alt_m(out.position_enu_m, origin_);

    out.speed_mps   = horizontal_speed(out.velocity_enu_mps);
    out.heading_deg = heading_deg_from_velocity(out.velocity_enu_mps);

    out.uncertainty.pos_horizontal_m =
        std::sqrt(std::max(0.0, P_[kPx][kPx] + P_[kPy][kPy]));
    out.uncertainty.pos_vertical_m = std::sqrt(std::max(0.0, P_[kPz][kPz]));
    out.uncertainty.vel_mps =
        std::sqrt(std::max(0.0, P_[kVx][kVx] + P_[kVy][kVy]));

    if (config_.enable_coast && initialized_) {
        const std::uint32_t gap_ms = elapsed_time_ms > last_gps_update_ms_
            ? elapsed_time_ms - last_gps_update_ms_
            : 0;
        out.coasting = gap_ms > config_.coast_after_gps_gap_ms;
    }

    return out;
}

std::vector<FusedState> fuse_gps_imu(
    const sf::proc::OrientedRide&       imu,
    const std::vector<gps::GPSReading>& gps,
    const gps::GeoOrigin&               origin,
    const EkfFusionConfig&              config)
{
    std::vector<FusedState> out;
    if (imu.samples.empty() || gps.empty()) {
        return out;
    }

    std::vector<gps::GPSReading> gps_sorted = gps;
    std::sort(gps_sorted.begin(), gps_sorted.end(),
        [](const gps::GPSReading& a, const gps::GPSReading& b) {
            return a.elapsed_time_ms < b.elapsed_time_ms;
        });

    const auto init_it = std::find_if(gps_sorted.begin(), gps_sorted.end(),
        [](const gps::GPSReading& r) { return r.have_enu; });
    if (init_it == gps_sorted.end()) {
        throw std::invalid_argument("fuse_gps_imu: no GPS fix with ENU position");
    }

    EkfFusion ekf(origin, config);
    ekf.initialize(*init_it);

    std::size_t gps_idx =
        static_cast<std::size_t>(std::distance(gps_sorted.begin(), init_it)) + 1;
    out.reserve(imu.samples.size());

    std::uint32_t prev_ms = imu.samples.front().elapsed_time_ms;

    for (const auto& sample : imu.samples) {
        while (gps_idx < gps_sorted.size()
               && gps_sorted[gps_idx].elapsed_time_ms <= sample.elapsed_time_ms) {
            ekf.update(gps_sorted[gps_idx]);
            ++gps_idx;
        }

        double dt_s = 0.0;
        if (sample.elapsed_time_ms > prev_ms) {
            dt_s = static_cast<double>(sample.elapsed_time_ms - prev_ms) * MS_TO_S;
        }
        prev_ms = sample.elapsed_time_ms;

        bool coasting = false;
        if (config.enable_coast && ekf.initialized()) {
            const std::uint32_t gap_ms =
                sample.elapsed_time_ms > ekf.last_gps_update_ms()
                    ? sample.elapsed_time_ms - ekf.last_gps_update_ms()
                    : 0;
            coasting = gap_ms > config.coast_after_gps_gap_ms;
        }

        ekf.predict(sample.accel_global, dt_s, coasting);
        FusedState state = ekf.snapshot(sample.elapsed_time_ms);
        state.coasting   = coasting;
        out.push_back(state);
    }

    return out;
}

} // namespace sf::fusion
