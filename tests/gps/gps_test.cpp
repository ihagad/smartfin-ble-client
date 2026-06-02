#include "geo_golden_vectors.hpp"
#include "gps/geo_utils.hpp"
#include "gps/sensorlog_parser.hpp"
#include "gps/session_align.hpp"
#include "pipeline/file_sink.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>

namespace {

constexpr double kOriginLat = 32.867512;
constexpr double kOriginLon = -117.256843;
constexpr double kOriginAlt = 2.3;

constexpr double kGoldenTolM = 1e-3;

constexpr std::int64_t kFixtureT0UnixMs = 1780324200LL * 1000LL;
constexpr std::uint32_t kAlignTolMs     = 50;

} // namespace

TEST(GpsSessionAlign, OverlapFractionUnion)
{
    const sf::gps::TimeSpanMs gps{1000, 5000};
    const sf::gps::TimeSpanMs imu{0, 4000};
    EXPECT_EQ(sf::gps::overlap_duration_ms(gps, imu), 3000u);
    EXPECT_NEAR(sf::gps::overlap_fraction(gps, imu), 3000.0 / 5000.0, 1e-9);
}

TEST(GpsSessionAlign, PairedFixturesOverlapAndSpan)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string gps_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";
    const std::string sfdat_path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/ride_20260601_143000.sfdat";

    auto gps  = sf::gps::load_sensorlog_csv(gps_path);
    auto ride = sf::pipeline::load_ride(sfdat_path);
    const auto imu_ms = sf::gps::collect_imu_elapsed_ms(ride);
    ASSERT_FALSE(gps.empty());
    ASSERT_FALSE(imu_ms.empty());

    const auto report =
        sf::gps::align_sessions(std::move(gps), imu_ms, kFixtureT0UnixMs);

    EXPECT_GE(report.overlap_fraction, sf::gps::kMinSessionOverlapFraction);
    EXPECT_TRUE(report.warnings.empty()) << report.warnings[0];

    EXPECT_LE(report.gps_span.start_ms, report.imu_span.end_ms);
    EXPECT_GE(report.gps_span.end_ms, report.imu_span.start_ms);
    EXPECT_NEAR(report.gps_span.start_ms, 0u, kAlignTolMs);
    EXPECT_NEAR(report.imu_span.start_ms, 0u, kAlignTolMs);

    EXPECT_GE(report.gps_span.start_ms, report.imu_span.start_ms);
    EXPECT_LE(report.gps_span.end_ms, report.imu_span.end_ms);

    for (std::size_t i = 0; i < report.gps.size(); i += 10) {
        const auto& fix = report.gps[i];
        const auto nearest = *std::min_element(
            imu_ms.begin(), imu_ms.end(), [&](std::uint32_t a, std::uint32_t b) {
                return std::abs(static_cast<int>(a) - static_cast<int>(fix.elapsed_time_ms))
                     < std::abs(static_cast<int>(b) - static_cast<int>(fix.elapsed_time_ms));
            });
        EXPECT_LE(
            std::abs(static_cast<int>(nearest) - static_cast<int>(fix.elapsed_time_ms)),
            static_cast<int>(kAlignTolMs));
    }
}

TEST(GpsSessionAlign, LowOverlapEmitsWarning)
{
    sf::gps::GPSReading early{};
    early.unix_s = 1780324200.0;
    sf::gps::GPSReading late{};
    late.unix_s = 1780324300.0; // 100 s span

    const std::vector<std::uint32_t> imu_ms{0, 1000, 2000, 3000}; // 0–3 s only

    const auto report = sf::gps::align_sessions(
        {early, late}, imu_ms, kFixtureT0UnixMs);

    EXPECT_LT(report.overlap_fraction, sf::gps::kMinSessionOverlapFraction);
    ASSERT_FALSE(report.warnings.empty());
}

TEST(GpsGeoGolden, LatLonToEnuMatchesPythonReference)
{
    const sf::gps::GeoOrigin origin{
        geo_golden::kOrigin.lat_deg,
        geo_golden::kOrigin.lon_deg,
        geo_golden::kOrigin.alt_m,
    };

    for (std::size_t i = 0; i < geo_golden::kSampleCount; ++i) {
        const auto& s = geo_golden::kSamples[i];
        const sf::gps::ENU enu =
            sf::gps::latlon_to_enu(s.lat_deg, s.lon_deg, s.alt_m, origin);

        EXPECT_NEAR(enu.x, s.east_m, kGoldenTolM)
            << s.trajectory << " idx=" << s.index;
        EXPECT_NEAR(enu.y, s.north_m, kGoldenTolM)
            << s.trajectory << " idx=" << s.index;
        EXPECT_NEAR(enu.z, s.up_m, kGoldenTolM)
            << s.trajectory << " idx=" << s.index;
    }
}

TEST(GpsGeoGolden, EnuToLatLonMatchesPythonReference)
{
    const sf::gps::GeoOrigin origin{
        geo_golden::kOrigin.lat_deg,
        geo_golden::kOrigin.lon_deg,
        geo_golden::kOrigin.alt_m,
    };

    for (std::size_t i = 0; i < geo_golden::kSampleCount; ++i) {
        const auto& s = geo_golden::kSamples[i];
        const sf::gps::ENU enu{s.east_m, s.north_m, s.up_m};
        const auto [lat, lon] = sf::gps::enu_to_latlon(enu, origin);
        const double alt = sf::gps::enu_to_alt_m(enu, origin);

        EXPECT_NEAR(lat, s.lat_deg, 1e-9) << s.trajectory << " idx=" << s.index;
        EXPECT_NEAR(lon, s.lon_deg, 1e-9) << s.trajectory << " idx=" << s.index;
        EXPECT_NEAR(alt, s.alt_m, kGoldenTolM)
            << s.trajectory << " idx=" << s.index;
    }
}

TEST(GpsGeoUtils, LatLonEnuRoundTrip)
{
    const sf::gps::GeoOrigin origin{kOriginLat, kOriginLon, kOriginAlt};
    const double lat = kOriginLat + 1e-5;
    const double lon = kOriginLon - 2e-5;
    const double alt = kOriginAlt + 0.5;

    const sf::gps::ENU enu = sf::gps::latlon_to_enu(lat, lon, alt, origin);
    const auto [lat2, lon2] = sf::gps::enu_to_latlon(enu, origin);
    const double alt2 = sf::gps::enu_to_alt_m(enu, origin);

    EXPECT_NEAR(lat, lat2, 1e-9);
    EXPECT_NEAR(lon, lon2, 1e-9);
    EXPECT_NEAR(alt, alt2, 1e-6);
}

// paddle_pause fixture: 180 s @ 1 Hz → 181 fixes (phase G validation).
constexpr std::size_t kFakeGpsExpectedRows = 181;

TEST(GpsValidationFake, ParserReadsFakeCsvRowCount)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string path =
        std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";
    const auto track = sf::gps::load_sensorlog_csv(path);
    EXPECT_EQ(track.size(), kFakeGpsExpectedRows);
}

TEST(GpsSensorLogParser, LoadsFakeFixture)
{
#ifndef SMARTFIN_REPO_ROOT
#error "SMARTFIN_REPO_ROOT must be defined by CMake"
#endif
    const std::string path = std::string(SMARTFIN_REPO_ROOT) + "/unprocessed/fake_gps.csv";

    sf::gps::SensorLogLoadOptions opts;
    opts.session_t0_unix = 1780324200.0;
    opts.enu_origin      = sf::gps::GeoOrigin{kOriginLat, kOriginLon, kOriginAlt};

    const auto track = sf::gps::load_sensorlog_csv(path, opts);
    EXPECT_EQ(track.size(), kFakeGpsExpectedRows);
    EXPECT_NEAR(track.front().t_elapsed_s, 0.0, 1e-3);
    EXPECT_TRUE(track.front().have_enu);
    EXPECT_GT(track.back().position_enu_m.y, 10.0);
    EXPECT_GT(track.back().t_elapsed_s, 60.0);

    std::size_t valid_speed = 0;
    for (const auto& r : track) {
        if (r.speed_valid()) {
            ++valid_speed;
        }
    }
    EXPECT_GT(valid_speed, 0u);
}
