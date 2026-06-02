/**
 * Phase E — EKF GPS–IMU fusion tests.
 */

#include "fusion/ekf_fusion.hpp"
#include "gps/geo_utils.hpp"
#include "gps/sensorlog_parser.hpp"
#include "gps/session_align.hpp"
#include "proccessing/ride_orient.hpp"
#include "pipeline/file_sink.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kOriginLat = 32.867512;
constexpr double kOriginLon = -117.256843;
constexpr double kOriginAlt = 2.3;

constexpr std::int64_t kFixtureT0UnixMs = 1780324200LL * 1000LL;

sf::gps::GPSReading make_gps_fix(
    std::uint32_t elapsed_ms,
    double east_m,
    double north_m,
    double speed_mps,
    double course_deg)
{
    sf::gps::GPSReading r{};
    r.elapsed_time_ms = elapsed_ms;
    r.t_elapsed_s     = static_cast<double>(elapsed_ms) * 1e-3;
    r.unix_s          = 1780324200.0 + r.t_elapsed_s;
    r.position_enu_m  = sf::gps::ENU{east_m, north_m, 0.0};
    r.have_enu        = true;
    r.horizontal_accuracy_m = 3.0;
    r.vertical_accuracy_m   = 5.0;
    r.speed_mps               = speed_mps;
    r.speed_accuracy_mps      = 0.5;
    r.course_deg              = course_deg;
    r.course_accuracy_deg     = 5.0;
    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};
    const auto [lat, lon] = sf::gps::enu_to_latlon(r.position_enu_m, origin);
    r.lat_deg             = lat;
    r.lon_deg             = lon;
    r.alt_m               = kOriginAlt;
    return r;
}

sf::proc::OrientedRide make_constant_velocity_imu(
    std::uint32_t duration_ms, std::uint32_t dt_ms)
{
    sf::proc::OrientedRide ride;
    for (std::uint32_t t = 0; t <= duration_ms; t += dt_ms) {
        sf::proc::OrientedSample s{};
        s.elapsed_time_ms = t;
        s.accel_global    = math3d::Vec3{};
        ride.samples.push_back(s);
    }
    return ride;
}

} // namespace

TEST(EkfFusion, StraightEastTracksGps)
{
    std::vector<sf::gps::GPSReading> gps;
    constexpr double kSpeed = 1.5;
    for (int i = 0; i <= 10; ++i) {
        gps.push_back(make_gps_fix(
            static_cast<std::uint32_t>(i * 1000),
            kSpeed * static_cast<double>(i),
            0.0,
            kSpeed,
            90.0));
    }

    const auto imu = make_constant_velocity_imu(10'000, 18);
    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};

    const auto fused =
        sf::fusion::fuse_gps_imu(imu, gps, origin);

    ASSERT_EQ(fused.size(), imu.samples.size());
    ASSERT_GT(fused.size(), 50u);

    const auto& last = fused.back();
    EXPECT_NEAR(last.position_enu_m.x, kSpeed * 10.0, 2.0);
    EXPECT_NEAR(last.velocity_enu_mps.x, kSpeed, 0.5);
    EXPECT_NEAR(last.speed_mps, kSpeed, 0.5);
    EXPECT_NEAR(last.heading_deg, 90.0, 5.0);
    EXPECT_GT(last.uncertainty.pos_horizontal_m, 0.0);
}

TEST(EkfFusion, CoastModeInflatesUncertainty)
{
    std::vector<sf::gps::GPSReading> gps{
        make_gps_fix(0, 0.0, 0.0, 0.0, -1.0),
        make_gps_fix(1000, 1.5, 0.0, 1.5, 90.0),
    };
    gps[0].speed_mps         = -1.0;
    gps[0].course_deg        = -1.0;
    gps[0].speed_accuracy_mps = -1.0;

    const auto imu = make_constant_velocity_imu(5000, 100);
    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};

    sf::fusion::EkfFusionConfig cfg;
    cfg.enable_coast             = true;
    cfg.coast_after_gps_gap_ms   = 500;
    cfg.coast_process_inflation  = 4.0;

    const auto fused = sf::fusion::fuse_gps_imu(imu, gps, origin, cfg);

    const auto mid = fused[fused.size() / 2];
    const auto end = fused.back();

    EXPECT_TRUE(end.coasting);
    EXPECT_GT(end.uncertainty.pos_horizontal_m, mid.uncertainty.pos_horizontal_m);
}

TEST(EkfFusion, FakeGpsAndSyntheticImuNonEmptyTrajectory)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string gps_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";
    const std::string sfdat_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};

    sf::gps::SensorLogLoadOptions opts;
    opts.session_t0_unix = 1780324200.0;
    opts.enu_origin      = origin;

    auto gps  = sf::gps::load_sensorlog_csv(gps_path, opts);
    auto ride = sf::pipeline::load_ride(sfdat_path);
    const auto imu_ms = sf::gps::collect_imu_elapsed_ms(ride);
    const auto aligned =
        sf::gps::align_sessions(std::move(gps), imu_ms, kFixtureT0UnixMs);

    sf::proc::Processor proc;
    const auto oriented = sf::proc::load_and_orient_ride(sfdat_path, proc);

    const auto fused = sf::fusion::fuse_gps_imu(oriented, aligned.gps, origin);

    ASSERT_FALSE(fused.empty());
    EXPECT_EQ(fused.size(), oriented.samples.size());
}

TEST(EkfFusion, FusedPositionErrorSmallVsGps)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string gps_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";
    const std::string sfdat_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};

    sf::gps::SensorLogLoadOptions opts;
    opts.session_t0_unix = 1780324200.0;
    opts.enu_origin      = origin;

    auto gps  = sf::gps::load_sensorlog_csv(gps_path, opts);
    auto ride = sf::pipeline::load_ride(sfdat_path);
    const auto imu_ms = sf::gps::collect_imu_elapsed_ms(ride);
    const auto aligned =
        sf::gps::align_sessions(std::move(gps), imu_ms, kFixtureT0UnixMs);

    sf::proc::Processor proc;
    const auto oriented = sf::proc::load_and_orient_ride(sfdat_path, proc);
    const auto fused    = sf::fusion::fuse_gps_imu(oriented, aligned.gps, origin);

    std::size_t checks = 0;
    double      sum_err = 0.0;
    double      max_err = 0.0;

    for (std::size_t i = 0; i < aligned.gps.size(); i += 5) {
        const auto& fix = aligned.gps[i];
        if (!fix.have_enu) {
            continue;
        }
        const auto it = std::min_element(
            fused.begin(), fused.end(),
            [&](const sf::fusion::FusedState& a, const sf::fusion::FusedState& b) {
                return std::abs(static_cast<int>(a.elapsed_time_ms)
                            - static_cast<int>(fix.elapsed_time_ms))
                     < std::abs(static_cast<int>(b.elapsed_time_ms)
                            - static_cast<int>(fix.elapsed_time_ms));
            });
        if (std::abs(static_cast<int>(it->elapsed_time_ms)
                   - static_cast<int>(fix.elapsed_time_ms))
            > 50) {
            continue;
        }
        const double de = it->position_enu_m.x - fix.position_enu_m.x;
        const double dn = it->position_enu_m.y - fix.position_enu_m.y;
        const double err = std::sqrt(de * de + dn * dn);
        sum_err += err;
        max_err  = std::max(max_err, err);
        ++checks;
    }

    ASSERT_GT(checks, 15u);
    EXPECT_LT(max_err, 6.0);
    EXPECT_LT(sum_err / static_cast<double>(checks), 2.0);
}

TEST(EkfFusion, PairedFixturesProduceDenseTrack)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string gps_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";
    const std::string sfdat_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};

    sf::gps::SensorLogLoadOptions opts;
    opts.session_t0_unix = 1780324200.0;
    opts.enu_origin      = origin;

    auto gps  = sf::gps::load_sensorlog_csv(gps_path, opts);
    auto ride = sf::pipeline::load_ride(sfdat_path);
    const auto imu_ms = sf::gps::collect_imu_elapsed_ms(ride);
    const auto aligned =
        sf::gps::align_sessions(std::move(gps), imu_ms, kFixtureT0UnixMs);

    sf::proc::Processor proc;
    const auto oriented = sf::proc::load_and_orient_ride(sfdat_path, proc);

    const auto fused = sf::fusion::fuse_gps_imu(
        oriented, aligned.gps, origin);

    ASSERT_EQ(fused.size(), oriented.samples.size());
    EXPECT_GT(fused.size(), 5000u);

    std::size_t gps_match_checks = 0;
    for (const auto& fix : aligned.gps) {
        if (!fix.have_enu) {
            continue;
        }
        const auto it = std::min_element(
            fused.begin(), fused.end(),
            [&](const sf::fusion::FusedState& a, const sf::fusion::FusedState& b) {
                return std::abs(static_cast<int>(a.elapsed_time_ms)
                            - static_cast<int>(fix.elapsed_time_ms))
                     < std::abs(static_cast<int>(b.elapsed_time_ms)
                            - static_cast<int>(fix.elapsed_time_ms));
            });
        ASSERT_NE(it, fused.end());
        if (std::abs(static_cast<int>(it->elapsed_time_ms)
                   - static_cast<int>(fix.elapsed_time_ms))
            > 50) {
            continue;
        }
        const double pos_err = std::sqrt(
            (it->position_enu_m.x - fix.position_enu_m.x)
                * (it->position_enu_m.x - fix.position_enu_m.x)
            + (it->position_enu_m.y - fix.position_enu_m.y)
                * (it->position_enu_m.y - fix.position_enu_m.y));
        const double tol =
            std::max(5.0, fix.horizontal_accuracy_m * 3.0);
        EXPECT_LT(pos_err, tol) << "gps elapsed_ms=" << fix.elapsed_time_ms;
        ++gps_match_checks;
    }
    EXPECT_GT(gps_match_checks, 10u);

    double max_north = 0.0;
    for (const auto& s : fused) {
        max_north = std::max(max_north, s.position_enu_m.y);
    }
    EXPECT_GT(max_north, 10.0);
}
