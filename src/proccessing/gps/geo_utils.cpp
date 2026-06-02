/**
 * @file geo_utils.cpp
 * @brief WGS84 lat/lon ↔ local ENU conversion.
 */

#include "gps/geo_utils.hpp"

#include "math/constants.hpp"

#include <cmath>

namespace sf::gps {

ENU latlon_to_enu(
    double lat_deg, double lon_deg, double alt_m, const GeoOrigin& origin)
{
    const double lat0_rad = origin.lat_deg * DEG_TO_RAD;
    const double dlat_rad = (lat_deg - origin.lat_deg) * DEG_TO_RAD;
    const double dlon_rad = (lon_deg - origin.lon_deg) * DEG_TO_RAD;

    const double north_m = dlat_rad * EARTH_RADIUS_M;
    const double east_m  = dlon_rad * EARTH_RADIUS_M * std::cos(lat0_rad);
    const double up_m    = alt_m - origin.alt_m;

    return ENU{east_m, north_m, up_m};
}

std::pair<double, double> enu_to_latlon(const ENU& enu_m, const GeoOrigin& origin)
{
    const double lat0_rad = origin.lat_deg * DEG_TO_RAD;
    const double dlat_rad = enu_m.y / EARTH_RADIUS_M;
    const double dlon_rad = enu_m.x / (EARTH_RADIUS_M * std::cos(lat0_rad));

    const double lat_deg = origin.lat_deg + dlat_rad * RAD_TO_DEG;
    const double lon_deg = origin.lon_deg + dlon_rad * RAD_TO_DEG;
    return {lat_deg, lon_deg};
}

double enu_to_alt_m(const ENU& enu_m, const GeoOrigin& origin)
{
    return origin.alt_m + enu_m.z;
}

void fill_enu_positions(std::vector<GPSReading>& track, const GeoOrigin& origin)
{
    for (auto& r : track) {
        r.position_enu_m = latlon_to_enu(r.lat_deg, r.lon_deg, r.alt_m, origin);
        r.have_enu       = true;
    }
}

} // namespace sf::gps
