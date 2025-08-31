#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>
#include <vector>

namespace ntrip_nmea_bridge {

class NmeaParser {
public:
    NmeaParser() = default;

    bool parseGGA(const std::string &line, sensor_msgs::msg::NavSatFix &fix);
    bool parseRMC(const std::string &line, geometry_msgs::msg::TwistStamped &twist, rclcpp::Time stamp);

private:
    static int splitCSV(const std::string &s, std::vector<std::string> &out);
    static bool nmeaToDeg(const std::string &field, char hemi, double &deg_out);
};

} // namespace ntrip_nmea_bridge
