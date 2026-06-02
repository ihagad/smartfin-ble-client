/**
 * Phase D — load_ride → orient_ride → OrientedSample with accel_global.
 */

#include "proccessing/ride_orient.hpp"
#include "pipeline/file_sink.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kFixtureDurationS   = 180.0;
constexpr double kNominalImuHz       = 55.0;
constexpr double kImuHzTolerance     = 1.5;
constexpr std::size_t kFixtureQuatSamples =
    static_cast<std::size_t>(kFixtureDurationS * kNominalImuHz) + 1;

double vec3_norm(const math3d::Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

} // namespace

TEST(AhrsOrient, LoadAndOrientMatchesManualPipeline)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    sf::proc::Processor proc;
    const auto wired = sf::proc::load_and_orient_ride(path, proc);

    auto data = sf::pipeline::load_ride(path);
    const auto manual = proc.orient_ride(data);

    ASSERT_EQ(wired.samples.size(), manual.samples.size());
    for (std::size_t i = 0; i < wired.samples.size(); ++i) {
        EXPECT_EQ(wired.samples[i].elapsed_time_ms, manual.samples[i].elapsed_time_ms);
        EXPECT_NEAR(vec3_norm(wired.samples[i].accel_global),
            vec3_norm(manual.samples[i].accel_global),
            1e-12);
    }
}

TEST(AhrsOrient, FixtureSampleCountAndRate)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    sf::proc::Processor proc;
    const auto ride = sf::proc::load_and_orient_ride(path, proc);

    ASSERT_EQ(ride.samples.size(), kFixtureQuatSamples);

    const double hz = sf::proc::estimate_sample_rate_hz(ride);
    EXPECT_NEAR(hz, kNominalImuHz, kImuHzTolerance);

    const auto ride_data = sf::pipeline::load_ride(path);
    EXPECT_EQ(ride.samples.size(), ride_data.quat_imu.size());
    EXPECT_TRUE(ride_data.imu.empty());
}

TEST(AhrsOrient, AccelGlobalPopulatedAndTimeSorted)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    sf::proc::Processor proc;
    const auto ride = sf::proc::load_and_orient_ride(path, proc);
    ASSERT_GT(ride.samples.size(), 100u);

    std::size_t nonzero = 0;
    for (std::size_t i = 0; i < ride.samples.size(); ++i) {
        const auto& s = ride.samples[i];
        EXPECT_GT(vec3_norm(s.accel_global), 0.0);
        if (vec3_norm(s.accel_global) > 1e-9) {
            ++nonzero;
        }
        if (i > 0) {
            EXPECT_GE(s.elapsed_time_ms, ride.samples[i - 1].elapsed_time_ms);
        }
    }
    EXPECT_EQ(nonzero, ride.samples.size());
}
