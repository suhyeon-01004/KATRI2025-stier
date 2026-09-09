#pragma once
#include <GeographicLib/UTMUPS.hpp>
#include <optional>

struct UTM { int zone; bool northp; float x; float y; };

constexpr double KR_LAT_MIN = 33.0;
constexpr double KR_LAT_MAX = 39.0;
constexpr double KR_LON_MIN = 124.0;
constexpr double KR_LON_MAX = 132.0;
static constexpr double ORIGIN_X = 302000.0;
static constexpr double ORIGIN_Y = 4120000.0;

inline float rawToDeg(int64_t raw) { return static_cast<double>(raw) / 1e7; }

inline std::pair<float, float> rawToUtm(int64_t lat_raw, int64_t lon_raw)
{
    const float lat_deg = rawToDeg(lat_raw);
    const float lon_deg = rawToDeg(lon_raw);

    int zone = 0; bool northp = true; double x, y, gamma, k;
    GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, zone, northp, x, y, gamma, k);
    x -= ORIGIN_X;
    y -= ORIGIN_Y;

    return std::make_pair(x, y);
}

inline std::pair<float, float> poseToUtm(double_t pose_x, double_t pose_y)
{
    const float x = static_cast<float>(pose_x - ORIGIN_X);
    const float y = static_cast<float>(pose_y - ORIGIN_Y);

    return std::make_pair(x, y);
}