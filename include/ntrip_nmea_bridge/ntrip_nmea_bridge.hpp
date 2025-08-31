#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <string>
#include <thread>
#include <atomic>
#include "nmea_parser.hpp"

namespace ntrip_nmea_bridge {

class NtripNmeaBridge : public rclcpp::Node {
public:
    explicit NtripNmeaBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~NtripNmeaBridge();

private:
    void readerThread();
    bool connectSocket();
    void closeSocket();

    // Params
    std::string host_;
    int port_;
    std::string frame_id_;
    bool publish_fix_;
    bool publish_twist_;

    // Socket
    int sockfd_{-1};
    std::thread thread_;
    std::atomic<bool> running_{false};

    // Publishers
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    // Parser
    NmeaParser parser_;
};

} // namespace ntrip_nmea_bridge
