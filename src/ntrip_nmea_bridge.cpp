#include "ntrip_nmea_bridge/ntrip_nmea_bridge.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <sstream>

namespace ntrip_nmea_bridge {

//------------------------- Constructor -------------------------
NtripNmeaBridge::NtripNmeaBridge(const rclcpp::NodeOptions &options)
: Node("ntrip_nmea_bridge", options)
{
    host_ = this->declare_parameter<std::string>("tcp_host", "127.0.0.1");
    port_ = this->declare_parameter<int>("tcp_port", 52001);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "gps");
    publish_fix_ = this->declare_parameter<bool>("publish_fix", true);
    publish_twist_ = this->declare_parameter<bool>("publish_twist", true);

    nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 100);
    if (publish_fix_) fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    if (publish_twist_) twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("ground_speed", 10);

    running_.store(true);
    thread_ = std::thread(&NtripNmeaBridge::readerThread, this);
}

//------------------------- Destructor -------------------------
NtripNmeaBridge::~NtripNmeaBridge() {
    running_.store(false);
    closeSocket();
    if (thread_.joinable()) thread_.join();
}

//------------------------- Socket -------------------------
bool NtripNmeaBridge::connectSocket() {
    closeSocket();
    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    std::string port_str = std::to_string(port_);
    if (getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &res) != 0) {
        RCLCPP_WARN(get_logger(), "getaddrinfo failed for %s:%d", host_.c_str(), port_);
        return false;
    }

    int s = -1;
    for (auto p = res; p != nullptr; p = p->ai_next) {
        s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (s == -1) continue;
        if (::connect(s, p->ai_addr, p->ai_addrlen) == 0) {
            sockfd_ = s;
            freeaddrinfo(res);
            RCLCPP_INFO(get_logger(), "Connected to %s:%d", host_.c_str(), port_);
            return true;
        }
        ::close(s);
    }
    freeaddrinfo(res);
    return false;
}

void NtripNmeaBridge::closeSocket() {
    if (sockfd_ >= 0) {
        ::shutdown(sockfd_, SHUT_RDWR);
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

//------------------------- Reader Thread -------------------------
void NtripNmeaBridge::readerThread() {
    std::string buffer;
    buffer.reserve(4096);
    int reconnect_delay = 1;

    while (running_.load()) {
        if (sockfd_ < 0) {
            if (!connectSocket()) {
                rclcpp::sleep_for(std::chrono::seconds(reconnect_delay));
                reconnect_delay = std::min(reconnect_delay*2, 16);
                continue;
            }
            reconnect_delay = 1;
        }

        char temp[1024];
        ssize_t n = ::recv(sockfd_, temp, sizeof(temp), 0);
        if (n <= 0) { closeSocket(); continue; }

        buffer.append(temp, temp+n);
        size_t pos = 0;
        const size_t max_buffer_size = 16*1024;

        while(true) {
            auto nl = buffer.find('\n', pos);
            if (nl == std::string::npos) {
                if (buffer.size() > max_buffer_size) buffer.clear();
                else buffer.erase(0, pos);
                break;
            }

            std::string line = buffer.substr(pos, nl-pos);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            pos = nl+1;
            if (line.empty()) continue;

            // Publish raw
            nmea_msgs::msg::Sentence sentence;
            sentence.header.stamp = this->now();
            sentence.header.frame_id = frame_id_;
            sentence.sentence = line;
            nmea_pub_->publish(sentence);

            if (publish_fix_ && line.rfind("$GNGGA",0)==0) {
                sensor_msgs::msg::NavSatFix fix;
                if (parser_.parseGGA(line, fix)) { fix.header = sentence.header; fix_pub_->publish(fix); }
            }
            else if (publish_twist_ && line.rfind("$GNRMC",0)==0) {
                geometry_msgs::msg::TwistStamped tw;
                if (parser_.parseRMC(line, tw, sentence.header.stamp)) { tw.header.frame_id = frame_id_; twist_pub_->publish(tw); }
            }
        }
    }
}

} // namespace ntrip_nmea_bridge

//------------------------- Main -------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ntrip_nmea_bridge::NtripNmeaBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
