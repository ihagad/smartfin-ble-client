/**
 * @file geo_utils.hpp
 * @brief WGS84 lat/lon ↔ local ENU conversion.
 */

#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "gps/gps_types.hpp"

#include <utility>
#include <vector>

namespace sf::gps {

/// Mean Earth radius (meters); matches tools/generate_fake_gps.py.
constexpr double EARTH_RADIUS_M = 6'371'000.0;

/**
 * @brief Convert WGS84 coordinates to ENU meters relative to @p origin.
 */
[[nodiscard]] ENU latlon_to_enu(
    double lat_deg, double lon_deg, double alt_m, const GeoOrigin& origin);

/**
 * @brief Convert ENU meters to WGS84 lat/lon; altitude from origin + Up.
 * @return (lat_deg, lon_deg).
 */
[[nodiscard]] std::pair<double, double> enu_to_latlon(
    const ENU& enu_m, const GeoOrigin& origin);

/**
 * @brief Altitude (m) from origin ellipsoid height plus ENU Up.
 */
[[nodiscard]] double enu_to_alt_m(const ENU& enu_m, const GeoOrigin& origin);

/**
 * @brief Fill @c position_enu_m on each reading using @p origin.
 */
void fill_enu_positions(std::vector<GPSReading>& track, const GeoOrigin& origin);

} // namespace sf::gps

#endif // GEO_UTILS_HPP
