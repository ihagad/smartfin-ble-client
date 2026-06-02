/**
 * @file gps_inspect.cpp
 * @brief Print summary stats for a SensorLog GPS CSV.
 *
 * Usage:
 *   gps_inspect <path.csv>
 *   gps_inspect <path.csv> --t0 <unix_s>
 */

#include "gps/geo_utils.hpp"
#include "gps/sensorlog_parser.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace {

void usage(const char* argv0)
{
    std::fprintf(stderr,
        "Usage: %s <sensorlog.csv> [--t0 <unix_epoch_s>]\n"
        "  --t0  Session start for elapsed-time stats (default: first fix)\n",
        argv0);
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string path = argv[1];
    double session_t0      = std::numeric_limits<double>::quiet_NaN();

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--t0" && i + 1 < argc) {
            session_t0 = std::stod(argv[++i]);
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    try {
        auto with_meta = sf::gps::load_sensorlog_csv(path);

        if (std::isnan(session_t0) && !with_meta.empty()) {
            session_t0 = with_meta.front().unix_s;
        }
        for (auto& r : with_meta) {
            r.t_elapsed_s = r.unix_s - session_t0;
        }
        const sf::gps::GeoOrigin origin{
            with_meta.front().lat_deg,
            with_meta.front().lon_deg,
            with_meta.front().alt_m,
        };
        sf::gps::fill_enu_positions(with_meta, origin);

        const double t_min = with_meta.front().unix_s;
        const double t_max = with_meta.back().unix_s;
        const double span  = t_max - t_min;

        double speed_min = std::numeric_limits<double>::infinity();
        double speed_max = -std::numeric_limits<double>::infinity();
        double speed_sum = 0.0;
        std::size_t speed_n = 0;

        double lat_min = with_meta.front().lat_deg;
        double lat_max = lat_min;
        double lon_min = with_meta.front().lon_deg;
        double lon_max = lon_min;

        double enu_east_max = 0.0;
        double enu_north_max = 0.0;

        for (const auto& r : with_meta) {
            lat_min = std::min(lat_min, r.lat_deg);
            lat_max = std::max(lat_max, r.lat_deg);
            lon_min = std::min(lon_min, r.lon_deg);
            lon_max = std::max(lon_max, r.lon_deg);

            if (r.have_enu) {
                enu_east_max  = std::max(enu_east_max, std::abs(r.position_enu_m.x));
                enu_north_max = std::max(enu_north_max, std::abs(r.position_enu_m.y));
            }

            if (!r.speed_valid()) {
                continue;
            }
            speed_min = std::min(speed_min, r.speed_mps);
            speed_max = std::max(speed_max, r.speed_mps);
            speed_sum += r.speed_mps;
            ++speed_n;
        }

        const double elapsed_span =
            with_meta.back().t_elapsed_s - with_meta.front().t_elapsed_s;

        std::printf("file: %s\n", path.c_str());
        std::printf("fixes: %zu\n", with_meta.size());
        std::printf("unix time: %.3f .. %.3f  (span %.1f s)\n", t_min, t_max, span);
        std::printf("session t0: %.0f\n", session_t0);
        std::printf("elapsed: %.3f .. %.3f s  (span %.1f s)\n",
            with_meta.front().t_elapsed_s,
            with_meta.back().t_elapsed_s,
            elapsed_span);
        std::printf("lat: %.6f .. %.6f  lon: %.6f .. %.6f\n",
            lat_min, lat_max, lon_min, lon_max);
        std::printf("ENU extent (origin=first fix): east ±%.1f m  north ±%.1f m\n",
            enu_east_max, enu_north_max);

        if (speed_n == 0) {
            std::printf("speed: no valid samples\n");
        } else {
            std::printf("speed (valid n=%zu): min %.3f  max %.3f  mean %.3f m/s\n",
                speed_n, speed_min, speed_max, speed_sum / static_cast<double>(speed_n));
        }

        const double dt_mean =
            with_meta.size() > 1
                ? span / static_cast<double>(with_meta.size() - 1)
                : 0.0;
        if (with_meta.size() > 1) {
            std::printf("mean fix interval: %.2f s (~%.2f Hz)\n",
                dt_mean, dt_mean > 0.0 ? 1.0 / dt_mean : 0.0);
        }

    } catch (const std::exception& ex) {
        std::fprintf(stderr, "gps_inspect: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
