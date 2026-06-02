/**
 * @file trajectory_export.cpp
 */

#include "fusion/trajectory_export.hpp"

#include "gps/geo_utils.hpp"
#include "proccessing/math/constants.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace sf::fusion {
namespace {

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

FusedState state_from_enu(
    std::uint32_t elapsed_ms,
    const gps::ENU& pos,
    const gps::ENU& vel,
    const gps::GeoOrigin& origin,
    bool coasting)
{
    FusedState s{};
    s.elapsed_time_ms  = elapsed_ms;
    s.t_elapsed_s      = static_cast<double>(elapsed_ms) * MS_TO_S;
    s.position_enu_m   = pos;
    s.velocity_enu_mps = vel;
    const auto [lat, lon] = gps::enu_to_latlon(pos, origin);
    s.lat_deg          = lat;
    s.lon_deg          = lon;
    s.alt_m            = gps::enu_to_alt_m(pos, origin);
    s.speed_mps        = horizontal_speed(vel);
    s.heading_deg      = heading_deg_from_velocity(vel);
    s.coasting         = coasting;
    return s;
}

} // namespace

std::vector<FusedState> dead_reckon_imu(
    const sf::proc::OrientedRide& imu,
    const gps::GPSReading&        seed,
    const gps::GeoOrigin&         origin,
    double                        max_predict_dt_s)
{
    if (!seed.have_enu || imu.samples.empty()) {
        return {};
    }

    math3d::Vec3 vel = math3d::Vec3{};
    if (seed.speed_valid() && seed.course_valid()) {
        const double course_rad = seed.course_deg * DEG_TO_RAD;
        vel.x                   = seed.speed_mps * std::sin(course_rad);
        vel.y                   = seed.speed_mps * std::cos(course_rad);
    }

    gps::ENU pos = seed.position_enu_m;

    std::vector<FusedState> out;
    out.reserve(imu.samples.size());

    std::uint32_t prev_ms = imu.samples.front().elapsed_time_ms;

    for (const auto& sample : imu.samples) {
        double dt_s = 0.0;
        if (sample.elapsed_time_ms > prev_ms) {
            dt_s = static_cast<double>(sample.elapsed_time_ms - prev_ms) * MS_TO_S;
            dt_s = std::min(dt_s, max_predict_dt_s);
        }
        prev_ms = sample.elapsed_time_ms;

        if (dt_s > 0.0) {
            const math3d::Vec3 a = sample.accel_global * G0;
            pos.x += vel.x * dt_s + 0.5 * a.x * dt_s * dt_s;
            pos.y += vel.y * dt_s + 0.5 * a.y * dt_s * dt_s;
            pos.z += vel.z * dt_s + 0.5 * a.z * dt_s * dt_s;
            vel.x += a.x * dt_s;
            vel.y += a.y * dt_s;
            vel.z += a.z * dt_s;
        }

        out.push_back(state_from_enu(
            sample.elapsed_time_ms, pos, vel, origin, false));
    }

    return out;
}

void write_trajectory_csv(
    const std::string&             path,
    const std::vector<FusedState>& track)
{
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("trajectory CSV: cannot open " + path);
    }

    out << std::setprecision(9);
    out << "elapsed_ms,t_elapsed_s,lat_deg,lon_deg,alt_m,"
           "east_m,north_m,up_m,ve,vn,vu,speed_mps,heading_deg,"
           "pos_sigma_h_m,pos_sigma_v_m,vel_sigma_mps,coasting\n";

    for (const auto& s : track) {
        out << s.elapsed_time_ms << ','
            << s.t_elapsed_s << ','
            << s.lat_deg << ','
            << s.lon_deg << ','
            << s.alt_m << ','
            << s.position_enu_m.x << ','
            << s.position_enu_m.y << ','
            << s.position_enu_m.z << ','
            << s.velocity_enu_mps.x << ','
            << s.velocity_enu_mps.y << ','
            << s.velocity_enu_mps.z << ','
            << s.speed_mps << ','
            << s.heading_deg << ','
            << s.uncertainty.pos_horizontal_m << ','
            << s.uncertainty.pos_vertical_m << ','
            << s.uncertainty.vel_mps << ','
            << (s.coasting ? 1 : 0) << '\n';
    }
}

} // namespace sf::fusion
