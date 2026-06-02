/**
 * @file sensorlog_parser.cpp
 * @brief Load SensorLog GPS CSV exports into @c GPSReading tracks.
 */

#include "gps/sensorlog_parser.hpp"

#include "gps/geo_utils.hpp"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace sf::gps {
namespace {

constexpr const char* COL_TIMESTAMP = "locationTimestamp_since1970(s)";
constexpr const char* COL_LAT       = "locationLatitude(WGS84)";
constexpr const char* COL_LON       = "locationLongitude(WGS84)";
constexpr const char* COL_ALT       = "locationAltitude(m)";
constexpr const char* COL_SPEED     = "locationSpeed(m/s)";
constexpr const char* COL_SPEED_ACC = "locationSpeedAccuracy(m/s)";
constexpr const char* COL_SPEED_ACC_DEG_TYPO = "locationSpeedAccuracy(°)";
constexpr const char* COL_COURSE    = "locationCourse(°)";
constexpr const char* COL_COURSE_ACC = "locationCourseAccuracy(°)";
constexpr const char* COL_VERT_ACC  = "locationVerticalAccuracy(m)";
constexpr const char* COL_HORZ_ACC  = "locationHorizontalAccuracy(m)";
constexpr const char* COL_FLOOR     = "locationFloor(Z)";

std::string trim(std::string s)
{
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> fields;
    std::string field;
    std::istringstream ss(line);
    while (std::getline(ss, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

void strip_utf8_bom(std::string& s)
{
    if (s.size() >= 3
        && static_cast<unsigned char>(s[0]) == 0xEF
        && static_cast<unsigned char>(s[1]) == 0xBB
        && static_cast<unsigned char>(s[2]) == 0xBF) {
        s.erase(0, 3);
    }
}

double parse_double(const std::string& field, const std::string& column, std::size_t row)
{
    try {
        std::size_t idx = 0;
        const double v  = std::stod(field, &idx);
        if (idx != field.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return v;
    } catch (const std::exception&) {
        throw std::runtime_error(
            "SensorLog CSV row " + std::to_string(row) + ": invalid number in "
            + column);
    }
}

std::int32_t parse_int32(const std::string& field, const std::string& column, std::size_t row)
{
    try {
        const long v = std::stol(field);
        return static_cast<std::int32_t>(v);
    } catch (const std::exception&) {
        throw std::runtime_error(
            "SensorLog CSV row " + std::to_string(row) + ": invalid integer in "
            + column);
    }
}

int column_index(
    const std::unordered_map<std::string, int>& cols, const char* name)
{
    const auto it = cols.find(name);
    if (it == cols.end()) {
        return -1;
    }
    return it->second;
}

double field_at(
    const std::vector<std::string>& row,
    int idx,
    const char* column,
    std::size_t row_num)
{
    if (idx < 0 || idx >= static_cast<int>(row.size())) {
        throw std::runtime_error(
            "SensorLog CSV row " + std::to_string(row_num) + ": missing " + column);
    }
    return parse_double(row[static_cast<std::size_t>(idx)], column, row_num);
}

GPSReading parse_row(
    const std::vector<std::string>& row,
    const std::unordered_map<std::string, int>& cols,
    std::size_t row_num)
{
    const int i_ts   = column_index(cols, COL_TIMESTAMP);
    const int i_lat  = column_index(cols, COL_LAT);
    const int i_lon  = column_index(cols, COL_LON);
    const int i_alt  = column_index(cols, COL_ALT);
    const int i_spd  = column_index(cols, COL_SPEED);
    int i_spd_acc    = column_index(cols, COL_SPEED_ACC);
    if (i_spd_acc < 0) {
        i_spd_acc = column_index(cols, COL_SPEED_ACC_DEG_TYPO);
    }
    const int i_crs  = column_index(cols, COL_COURSE);
    const int i_crs_acc = column_index(cols, COL_COURSE_ACC);
    const int i_vacc = column_index(cols, COL_VERT_ACC);
    const int i_hacc = column_index(cols, COL_HORZ_ACC);
    const int i_floor = column_index(cols, COL_FLOOR);

    if (i_ts < 0 || i_lat < 0 || i_lon < 0 || i_alt < 0 || i_spd < 0) {
        throw std::runtime_error(
            "SensorLog CSV: missing required location columns (see docs/SENSORLOG_CSV.md)");
    }

    GPSReading r;
    r.unix_s = field_at(row, i_ts, COL_TIMESTAMP, row_num);
    r.lat_deg = field_at(row, i_lat, COL_LAT, row_num);
    r.lon_deg = field_at(row, i_lon, COL_LON, row_num);
    r.alt_m   = field_at(row, i_alt, COL_ALT, row_num);
    r.speed_mps = field_at(row, i_spd, COL_SPEED, row_num);

    if (i_spd_acc >= 0) {
        r.speed_accuracy_mps =
            field_at(row, i_spd_acc, COL_SPEED_ACC, row_num);
    }
    if (i_crs >= 0) {
        r.course_deg = field_at(row, i_crs, COL_COURSE, row_num);
    }
    if (i_crs_acc >= 0) {
        r.course_accuracy_deg =
            field_at(row, i_crs_acc, COL_COURSE_ACC, row_num);
    }
    if (i_vacc >= 0) {
        r.vertical_accuracy_m =
            field_at(row, i_vacc, COL_VERT_ACC, row_num);
    }
    if (i_hacc >= 0) {
        r.horizontal_accuracy_m =
            field_at(row, i_hacc, COL_HORZ_ACC, row_num);
    }
    if (i_floor >= 0) {
        r.floor_z = parse_int32(
            row[static_cast<std::size_t>(i_floor)], COL_FLOOR, row_num);
    }

    return r;
}

void apply_load_options(
    std::vector<GPSReading>& track, const SensorLogLoadOptions& opts)
{
    if (opts.session_t0_unix) {
        const double t0 = *opts.session_t0_unix;
        const auto t0_ms = static_cast<std::int64_t>(std::llround(t0 * 1000.0));
        for (auto& r : track) {
            r.t_elapsed_s = r.unix_s - t0;
            const auto gps_ms = static_cast<std::int64_t>(std::llround(r.unix_s * 1000.0));
            const auto elapsed_ms = gps_ms - t0_ms;
            if (elapsed_ms >= 0
                && elapsed_ms <= static_cast<std::int64_t>(
                       std::numeric_limits<std::uint32_t>::max())) {
                r.elapsed_time_ms =
                    static_cast<std::uint32_t>(elapsed_ms);
            }
        }
    }
    if (opts.enu_origin) {
        fill_enu_positions(track, *opts.enu_origin);
    }
}

} // namespace

std::vector<GPSReading> load_sensorlog_csv(
    const std::string& path, const SensorLogLoadOptions& opts)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("SensorLog CSV: cannot open " + path);
    }

    std::string header_line;
    if (!std::getline(in, header_line)) {
        throw std::runtime_error("SensorLog CSV: empty file " + path);
    }
    strip_utf8_bom(header_line);

    const auto header_fields = split_csv_line(header_line);
    std::unordered_map<std::string, int> cols;
    cols.reserve(header_fields.size());
    for (int i = 0; i < static_cast<int>(header_fields.size()); ++i) {
        cols.emplace(header_fields[static_cast<std::size_t>(i)], i);
    }

    std::vector<GPSReading> track;
    std::string line;
    std::size_t row_num = 1;
    while (std::getline(in, line)) {
        ++row_num;
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto fields = split_csv_line(line);
        track.push_back(parse_row(fields, cols, row_num));
    }

    if (track.empty()) {
        throw std::runtime_error("SensorLog CSV: no data rows in " + path);
    }

    std::sort(track.begin(), track.end(), [](const GPSReading& a, const GPSReading& b) {
        return a.unix_s < b.unix_s;
    });

    apply_load_options(track, opts);
    return track;
}

} // namespace sf::gps
