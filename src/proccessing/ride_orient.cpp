/**
 * @file ride_orient.cpp
 * @brief Wire @c load_ride() to @c Processor::orient_ride().
 */

#include "proccessing/ride_orient.hpp"

#include <algorithm>
#include <vector>

namespace sf::proc {

OrientedRide load_and_orient_ride(const std::string& path, Processor& processor)
{
    sf::pipeline::RideData data = sf::pipeline::load_ride(path);
    return processor.orient_ride(data);
}

double estimate_sample_rate_hz(const OrientedRide& ride)
{
    const auto& samples = ride.samples;
    if (samples.size() < 2) {
        return 0.0;
    }

    std::vector<double> dt_s;
    dt_s.reserve(samples.size() - 1);
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const auto delta = samples[i].elapsed_time_ms - samples[i - 1].elapsed_time_ms;
        if (delta > 0) {
            dt_s.push_back(static_cast<double>(delta) * 1e-3);
        }
    }
    if (dt_s.empty()) {
        return 0.0;
    }

    const auto mid = dt_s.begin() + static_cast<std::ptrdiff_t>(dt_s.size() / 2);
    std::nth_element(dt_s.begin(), mid, dt_s.end());
    const double median_dt = *mid;
    return median_dt > 0.0 ? 1.0 / median_dt : 0.0;
}

} // namespace sf::proc
