/**
 * @file orient_ride_inspect.cpp
 * @brief Summary stats for load_ride → orient_ride on a .sfdat file.
 *
 * Usage: orient_ride_inspect <path.sfdat>
 */

#include "proccessing/ride_orient.hpp"
#include "proccessing/math/lin_alg.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace {

double vec3_norm(const math3d::Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::fprintf(stderr, "Usage: %s <ride.sfdat>\n", argv[0]);
        return EXIT_FAILURE;
    }

    try {
        sf::proc::Processor proc;
        const auto ride = sf::proc::load_and_orient_ride(argv[1], proc);
        const auto& samples = ride.samples;

        if (samples.empty()) {
            std::printf("file: %s\n", argv[1]);
            std::printf("oriented samples: 0\n");
            return EXIT_SUCCESS;
        }

        const double hz = sf::proc::estimate_sample_rate_hz(ride);
        const double duration_s =
            (samples.back().elapsed_time_ms - samples.front().elapsed_time_ms) * 1e-3;

        double accel_min = std::numeric_limits<double>::infinity();
        double accel_max = -std::numeric_limits<double>::infinity();
        std::size_t nonzero_accel = 0;

        for (const auto& s : samples) {
            const double mag = vec3_norm(s.accel_global);
            if (mag > 1e-12) {
                ++nonzero_accel;
            }
            accel_min = std::min(accel_min, mag);
            accel_max = std::max(accel_max, mag);
        }

        std::printf("file: %s\n", argv[1]);
        std::printf("oriented samples: %zu\n", samples.size());
        std::printf("elapsed: %u .. %u ms  (%.1f s)\n",
            samples.front().elapsed_time_ms,
            samples.back().elapsed_time_ms,
            duration_s);
        std::printf("median rate: %.2f Hz  (nominal %.0f Hz)\n",
            hz, sf::proc::kNominalImuRateHz);
        std::printf("accel_global |a| (g): min %.4f  max %.4f  nonzero %zu/%zu\n",
            accel_min, accel_max, nonzero_accel, samples.size());

    } catch (const std::exception& ex) {
        std::fprintf(stderr, "orient_ride_inspect: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
