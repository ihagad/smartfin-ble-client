/**
 * @file fusion_export.cpp
 * @brief Run GPS–IMU fusion on paired fixtures and write trajectory CSVs.
 *
 * Usage:
 *   fusion_export <ride.sfdat> <sensorlog.csv> --t0 <unix_s> [--out <dir>]
 */

#include "fusion/ekf_fusion.hpp"
#include "fusion/trajectory_export.hpp"
#include "gps/geo_utils.hpp"
#include "gps/sensorlog_parser.hpp"
#include "gps/session_align.hpp"
#include "proccessing/ride_orient.hpp"
#include "proccessing/math/constants.hpp"
#include "proccessing/math/lin_alg.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

void usage(const char* argv0)
{
    std::fprintf(stderr,
        "Usage: %s <ride.sfdat> <sensorlog.csv> --t0 <unix_epoch_s> [--out <dir>]\n",
        argv0);
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 5) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string sfdat_path = argv[1];
    const std::string gps_path   = argv[2];
    double            t0_unix    = 0.0;
    std::string       out_dir    = ".";

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--t0" && i + 1 < argc) {
            t0_unix = std::atof(argv[++i]);
        } else if (arg == "--out" && i + 1 < argc) {
            out_dir = argv[++i];
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (t0_unix <= 0.0) {
        std::fprintf(stderr, "--t0 <unix_epoch_s> is required\n");
        return EXIT_FAILURE;
    }

    try {
        sf::gps::SensorLogLoadOptions opts;
        opts.session_t0_unix = t0_unix;

        auto gps = sf::gps::load_sensorlog_csv(gps_path, opts);

        const auto first = gps.front();
        opts.enu_origin  = sf::gps::GeoOrigin{
            first.lat_deg, first.lon_deg, first.alt_m};
        sf::gps::fill_enu_positions(gps, *opts.enu_origin);

        auto ride   = sf::pipeline::load_ride(sfdat_path);
        const auto imu_ms = sf::gps::collect_imu_elapsed_ms(ride);
        const auto t0_ms  = static_cast<std::int64_t>(std::llround(t0_unix * 1000.0));
        const auto aligned =
            sf::gps::align_sessions(std::move(gps), imu_ms, t0_ms);

        sf::proc::Processor proc;
        const auto oriented = sf::proc::load_and_orient_ride(sfdat_path, proc);

        const auto fused = sf::fusion::fuse_gps_imu(
            oriented, aligned.gps, *opts.enu_origin);

        const auto& seed = *std::find_if(
            aligned.gps.begin(), aligned.gps.end(),
            [](const sf::gps::GPSReading& r) { return r.have_enu; });

        const auto imu_dr = sf::fusion::dead_reckon_imu(
            oriented, seed, *opts.enu_origin);

        std::filesystem::create_directories(out_dir);

        const std::string fused_path = out_dir + "/trajectory_fused.csv";
        const std::string imu_path   = out_dir + "/trajectory_imu_dr.csv";
        const std::string gps_out    = out_dir + "/trajectory_gps.csv";

        sf::fusion::write_trajectory_csv(fused_path, fused);
        sf::fusion::write_trajectory_csv(imu_path, imu_dr);

        std::vector<sf::fusion::FusedState> gps_track;
        gps_track.reserve(aligned.gps.size());
        for (const auto& fix : aligned.gps) {
            if (!fix.have_enu) {
                continue;
            }
            sf::fusion::FusedState row{};
            row.elapsed_time_ms  = fix.elapsed_time_ms;
            row.t_elapsed_s      = fix.t_elapsed_s;
            row.position_enu_m   = fix.position_enu_m;
            row.lat_deg          = fix.lat_deg;
            row.lon_deg          = fix.lon_deg;
            row.alt_m            = fix.alt_m;
            if (fix.speed_valid() && fix.course_valid()) {
                const double cr = fix.course_deg * DEG_TO_RAD;
                row.velocity_enu_mps.x = fix.speed_mps * std::sin(cr);
                row.velocity_enu_mps.y = fix.speed_mps * std::cos(cr);
                row.speed_mps          = fix.speed_mps;
                row.heading_deg        = fix.course_deg;
            } else {
                row.speed_mps   = 0.0;
                row.heading_deg = -1.0;
            }
            gps_track.push_back(row);
        }
        sf::fusion::write_trajectory_csv(gps_out, gps_track);

        const std::string accel_path = out_dir + "/imu_accel.csv";
        {
            std::ofstream accel_out(accel_path);
            if (!accel_out) {
                throw std::runtime_error("cannot open " + accel_path);
            }
            accel_out << "elapsed_ms,t_elapsed_s,accel_mag_g\n";
            for (const auto& s : oriented.samples) {
                const double mag = std::sqrt(
                    s.accel_global.x * s.accel_global.x
                    + s.accel_global.y * s.accel_global.y
                    + s.accel_global.z * s.accel_global.z);
                accel_out << s.elapsed_time_ms << ','
                          << static_cast<double>(s.elapsed_time_ms) * MS_TO_S
                          << ',' << mag << '\n';
            }
        }

        std::printf("Wrote %zu fused, %zu imu_dr, %zu gps rows\n",
            fused.size(), imu_dr.size(), gps_track.size());
        std::printf("  %s\n  %s\n  %s\n  %s\n",
            fused_path.c_str(), imu_path.c_str(), gps_out.c_str(),
            accel_path.c_str());

        for (const auto& w : aligned.warnings) {
            std::fprintf(stderr, "[fusion_export] %s\n", w.c_str());
        }
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "fusion_export: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
