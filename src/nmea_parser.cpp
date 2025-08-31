#include "ntrip_nmea_bridge/nmea_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>

namespace ntrip_nmea_bridge {

//------------------------- Helper CSV Split -------------------------
int NmeaParser::splitCSV(const std::string &s, std::vector<std::string> &out) {
    out.clear();
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return static_cast<int>(out.size());
}

//------------------------- NMEA → Degrees -------------------------
bool NmeaParser::nmeaToDeg(const std::string &field, char hemi, double &deg_out) {
    if (field.size() < 4) return false;
    size_t dot = field.find('.');
    size_t deg_len = (dot != std::string::npos ? (dot >= 4 ? dot - 2 : 0) : field.size() - 2);
    if (deg_len < 2 || deg_len > 3) return false;

    double deg = std::stod(field.substr(0, deg_len));
    double min = std::stod(field.substr(deg_len));
    deg_out = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') deg_out = -deg_out;
    return true;
}

//------------------------- Parse GGA -------------------------
bool NmeaParser::parseGGA(const std::string &line, sensor_msgs::msg::NavSatFix &fix) {
    std::vector<std::string> f;
    splitCSV(line, f);
    if (f.size() < 15) return false;

    double lat=0, lon=0, alt=0;
    if (!nmeaToDeg(f[2], f[3].empty()? 'N' : f[3][0], lat)) return false;
    if (!nmeaToDeg(f[4], f[5].empty()? 'E' : f[5][0], lon)) return false;
    try { alt = std::stod(f[9]); } catch (...) { alt = 0.0; }

    int quality = 0;
    try { quality = std::stoi(f[6]); } catch (...) {}

    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = alt;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix.status.status = (quality == 0) ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
                                       : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    return true;
}

//------------------------- Parse RMC -------------------------
bool NmeaParser::parseRMC(const std::string &line, geometry_msgs::msg::TwistStamped &twist, rclcpp::Time stamp) {
    std::vector<std::string> f;
    splitCSV(line, f);
    if (f.size() < 9) return false;

    if (f[2] != "A") return false; // valid flag

    double spd_kn = 0.0, cog_deg = 0.0;
    try { spd_kn = std::stod(f[7]); } catch (...) {}
    try { cog_deg = std::stod(f[8]); } catch (...) {}

    double spd_ms = spd_kn * 0.514444;
    double cog_rad = cog_deg * M_PI / 180.0;

    twist.header.stamp = stamp;
    twist.twist.linear.x = spd_ms * std::cos(cog_rad);
    twist.twist.linear.y = spd_ms * std::sin(cog_rad);
    twist.twist.linear.z = 0.0;
    return true;
}

} // namespace ntrip_nmea_bridge
