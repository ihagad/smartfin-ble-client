/**
 * @file session_align.cpp
 * @brief Align SensorLog GPS timestamps to fin IMU @c elapsed_time_ms.
 */

#include "gps/session_align.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>

namespace sf::gps {
namespace {

void warn(SessionAlignmentReport& report, const char* message)
{
    report.warnings.emplace_back(message);
    std::fprintf(stderr, "[gps_align] WARNING: %s\n", message);
}

TimeSpanMs span_from_elapsed(const std::vector<std::uint32_t>& elapsed_ms)
{
    if (elapsed_ms.empty()) {
        return {};
    }
    const auto [min_it, max_it] = std::minmax_element(
        elapsed_ms.begin(), elapsed_ms.end());
    return {*min_it, *max_it};
}

TimeSpanMs span_from_gps(const std::vector<GPSReading>& gps)
{
    if (gps.empty()) {
        return {};
    }
    return {gps.front().elapsed_time_ms, gps.back().elapsed_time_ms};
}

std::int64_t unix_s_to_ms(double unix_s)
{
    return static_cast<std::int64_t>(std::llround(unix_s * 1000.0));
}

} // namespace

std::vector<std::uint32_t> collect_imu_elapsed_ms(const sf::pipeline::RideData& ride)
{
    std::vector<std::uint32_t> out;
    out.reserve(ride.imu.size() + ride.quat_imu.size());
    for (const auto& s : ride.imu) {
        out.push_back(s.elapsed_time_ms);
    }
    for (const auto& s : ride.quat_imu) {
        out.push_back(s.elapsed_time_ms);
    }
    return out;
}

std::uint32_t overlap_duration_ms(TimeSpanMs a, TimeSpanMs b)
{
    const std::uint32_t start = std::max(a.start_ms, b.start_ms);
    const std::uint32_t end   = std::min(a.end_ms, b.end_ms);
    if (end < start) {
        return 0;
    }
    return end - start;
}

double overlap_fraction(TimeSpanMs gps, TimeSpanMs imu)
{
    const std::uint32_t overlap = overlap_duration_ms(gps, imu);
    const std::uint32_t union_start = std::min(gps.start_ms, imu.start_ms);
    const std::uint32_t union_end   = std::max(gps.end_ms, imu.end_ms);
    if (union_end < union_start) {
        return 0.0;
    }
    const std::uint32_t union_ms = union_end - union_start;
    if (union_ms == 0) {
        return overlap > 0 ? 1.0 : 0.0;
    }
    return static_cast<double>(overlap) / static_cast<double>(union_ms);
}

SessionAlignmentReport align_sessions(
    std::vector<GPSReading> gps,
    const std::vector<std::uint32_t>& imu_elapsed_ms,
    const std::int64_t t0_unix_ms)
{
    SessionAlignmentReport report;
    report.gps = std::move(gps);

    if (report.gps.empty()) {
        warn(report, "GPS track is empty");
        return report;
    }
    if (imu_elapsed_ms.empty()) {
        warn(report, "IMU elapsed_time_ms list is empty");
        return report;
    }

    std::sort(report.gps.begin(), report.gps.end(), [](const GPSReading& a,
                                                         const GPSReading& b) {
        return a.unix_s < b.unix_s;
    });

    for (auto& fix : report.gps) {
        const std::int64_t gps_unix_ms = unix_s_to_ms(fix.unix_s);
        const std::int64_t elapsed_ms  = gps_unix_ms - t0_unix_ms;
        if (elapsed_ms < 0) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GPS fix at unix %.3f maps to negative elapsed %lld ms",
                fix.unix_s, static_cast<long long>(elapsed_ms));
            warn(report, buf);
        }
        if (elapsed_ms > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
            warn(report, "GPS fix elapsed_time_ms overflows uint32");
            fix.elapsed_time_ms = std::numeric_limits<std::uint32_t>::max();
        } else if (elapsed_ms < 0) {
            fix.elapsed_time_ms = 0;
        } else {
            fix.elapsed_time_ms = static_cast<std::uint32_t>(elapsed_ms);
        }
        fix.t_elapsed_s = static_cast<double>(fix.elapsed_time_ms) * 1e-3;
    }

    report.imu_span = span_from_elapsed(imu_elapsed_ms);
    report.gps_span = span_from_gps(report.gps);
    report.overlap_fraction = overlap_fraction(report.gps_span, report.imu_span);

    if (overlap_duration_ms(report.gps_span, report.imu_span) == 0) {
        warn(report, "GPS and IMU elapsed_time_ms ranges do not overlap");
    } else {
        if (report.gps_span.start_ms < report.imu_span.start_ms) {
            warn(report,
                "First GPS elapsed_time_ms is before first IMU sample");
        }
        if (report.gps_span.end_ms > report.imu_span.end_ms) {
            warn(report,
                "Last GPS elapsed_time_ms is after last IMU sample");
        }
    }

    if (report.overlap_fraction < kMinSessionOverlapFraction) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "GPS/IMU timeline overlap %.1f%% < %.0f%% of session",
            report.overlap_fraction * 100.0,
            kMinSessionOverlapFraction * 100.0);
        warn(report, buf);
    }

    return report;
}

} // namespace sf::gps
