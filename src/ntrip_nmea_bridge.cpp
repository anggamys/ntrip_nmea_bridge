#include "ntrip_nmea_bridge/ntrip_nmea_bridge.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <vector>
#include <cmath>

namespace ntrip_nmea_bridge {

static int splitCSV(const std::string &s, std::vector<std::string> &out) {
  out.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(item);
  return static_cast<int>(out.size());
}

NtripNmeaBridge::NtripNmeaBridge(const rclcpp::NodeOptions &options)
: Node("ntrip_nmea_bridge", options)
{
  host_ = this->declare_parameter<std::string>("tcp_host", "127.0.0.1");
  port_ = this->declare_parameter<int>("tcp_port", 52001);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "gps");
  publish_fix_ = this->declare_parameter<bool>("publish_fix", true);
  publish_twist_ = this->declare_parameter<bool>("publish_twist", true);

  nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 100);
  if (publish_fix_) {
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
  }
  if (publish_twist_) {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("ground_speed", 10);
  }

  running_.store(true);
  thread_ = std::thread(&NtripNmeaBridge::readerThread, this);
}

NtripNmeaBridge::~NtripNmeaBridge() {
  running_.store(false);
  closeSocket();
  if (thread_.joinable()) thread_.join();
}

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

void NtripNmeaBridge::readerThread() {
  std::string buffer;
  buffer.reserve(4096);

  while (running_.load()) {
    if (sockfd_ < 0) {
      if (!connectSocket()) {
        rclcpp::sleep_for(std::chrono::seconds(2));
        continue;
      }
    }

    char temp[1024];
    ssize_t n = ::recv(sockfd_, temp, sizeof(temp), 0);
    if (n <= 0) {
      RCLCPP_WARN(get_logger(), "Socket recv error/closed (%ld). Reconnecting...", (long)n);
      closeSocket();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;
    }
    buffer.append(temp, temp + n);

    size_t pos = 0;
    while (true) {
      auto nl = buffer.find('\n', pos);
      if (nl == std::string::npos) {
        buffer.erase(0, pos); // simpan sisa
        break;
      }
      std::string line = buffer.substr(pos, nl - pos);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      pos = nl + 1;

      if (line.empty()) continue;

      // publish raw
      nmea_msgs::msg::Sentence sentence;
      sentence.header.stamp = this->now();
      sentence.header.frame_id = frame_id_;
      sentence.sentence = line;
      nmea_pub_->publish(sentence);

      // parse GGA → NavSatFix
      if (publish_fix_ && line.rfind("$GNGGA", 0) == 0) {
        sensor_msgs::msg::NavSatFix fix;
        if (parseGGA(line, fix)) {
          fix.header = sentence.header;
          fix_pub_->publish(fix);
        }
      }
      // parse RMC → TwistStamped (kecepatan darat)
      else if (publish_twist_ && line.rfind("$GNRMC", 0) == 0) {
        geometry_msgs::msg::TwistStamped tw;
        if (parseRMC(line, tw, sentence.header.stamp)) {
          tw.header.frame_id = frame_id_;
          twist_pub_->publish(tw);
        }
      }
    }
  }
}

bool NtripNmeaBridge::nmeaToDeg(const std::string &field, char hemi, double &deg_out) {
  // format: ddmm.mmmm atau dddmm.mmmm
  if (field.size() < 4) return false;
  size_t dot = field.find('.');
  size_t deg_len = (dot != std::string::npos ? (dot >= 4 ? dot - 2 : 0) : field.size() - 2);
  if (deg_len < 2 || deg_len > 3) return false;

  double deg = std::stod(field.substr(0, deg_len));
  double min = std::stod(field.substr(deg_len));
  double dec = deg + (min / 60.0);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  deg_out = dec;
  return true;
}

bool NtripNmeaBridge::parseGGA(const std::string &line, sensor_msgs::msg::NavSatFix &fix) {
  // $GNGGA,hhmmss.ss,lat,NS,lon,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
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

  // status sederhana
  if (quality == 0) {
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else {
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  return true;
}

bool NtripNmeaBridge::parseRMC(const std::string &line, geometry_msgs::msg::TwistStamped &twist, rclcpp::Time stamp) {
  // $GNRMC,hhmmss.ss,A,lat,NS,lon,EW,spd(kn),cog,ddmmyy,magVar,magEW,mode,navStat*cs
  std::vector<std::string> f;
  splitCSV(line, f);
  if (f.size() < 9) return false;

  double spd_kn = 0.0, cog_deg = 0.0;
  try { spd_kn = std::stod(f[7]); } catch (...) { spd_kn = 0.0; }
  try { cog_deg = std::stod(f[8]); } catch (...) { cog_deg = 0.0; }

  double spd_ms = spd_kn * 0.514444; // knots → m/s
  double cog_rad = cog_deg * M_PI / 180.0;

  twist.header.stamp = stamp;
  twist.twist.linear.x = spd_ms * std::cos(cog_rad);
  twist.twist.linear.y = spd_ms * std::sin(cog_rad);
  twist.twist.linear.z = 0.0;
  return true;
}

} // namespace ntrip_nmea_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ntrip_nmea_bridge::NtripNmeaBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
