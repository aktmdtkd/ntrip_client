// Copyright 2023 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <asio.hpp>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ntrip_client/visibility_control.h"

using namespace std::chrono_literals;

namespace ntrip_client
{
class NTRIPClientNode : public rclcpp::Node
{
public:
  NTRIP_CLIENT_NODE_PUBLIC
  explicit NTRIPClientNode(const rclcpp::NodeOptions & options)
  : Node("ntrip_client", options),
    io_context_(), socket_(io_context_)
  {
    RCLCPP_INFO(this->get_logger(), "starting ntrip_client");

    declare_parameter("host", "");
    declare_parameter("port",);
    declare_parameter("mountpoint", "");
    declare_parameter("username", "");
    declare_parameter("password", "");

    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    mountpoint_ = get_parameter("mountpoint").as_string();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();

    rtcm_pub_ = this->create_publisher<rtcm_msgs::msg::Message>("/rtcm", 10);
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix", 10,
      std::bind(&NTRIPClientNode::fixCallback, this, std::placeholders::_1));

    streaming_thread_ = std::thread(&NTRIPClientNode::streamRTCM, this);
  }

  ~NTRIPClientNode()
  {
    io_context_.stop();
    if (streaming_thread_.joinable()) {
      streaming_thread_.join();
    }
  }

private:
  std::string host_, mountpoint_, username_, password_;
  int port_;
  asio::io_context io_context_;
  asio::ip::tcp::socket socket_;
  std::thread streaming_thread_;

  std::mutex fix_mutex_;
  sensor_msgs::msg::NavSatFix::SharedPtr latest_fix_;

  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    latest_fix_ = msg;
  }

  std::string generateGGA()
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    if (!latest_fix_) return "";

    const auto & fix = *latest_fix_;
    std::time_t t = std::time(nullptr);
    std::tm * now = std::gmtime(&t);
    char time_str[16];
    std::strftime(time_str, sizeof(time_str), "%H%M%S", now);

    char lat_dir = fix.latitude >= 0 ? 'N' : 'S';
    double lat_deg = std::abs(fix.latitude);
    int lat_deg_int = static_cast<int>(lat_deg);
    double lat_min = (lat_deg - lat_deg_int) * 60.0;

    char lon_dir = fix.longitude >= 0 ? 'E' : 'W';
    double lon_deg = std::abs(fix.longitude);
    int lon_deg_int = static_cast<int>(lon_deg);
    double lon_min = (lon_deg - lon_deg_int) * 60.0;

    std::ostringstream ss;
    ss << "$GPGGA," << time_str << ","
       << std::setw(2) << std::setfill('0') << lat_deg_int
       << std::fixed << std::setprecision(4) << lat_min << "," << lat_dir << ","
       << std::setw(3) << std::setfill('0') << lon_deg_int
       << std::fixed << std::setprecision(4) << lon_min << "," << lon_dir
       << ",1,12,1.0," << std::fixed << std::setprecision(1) << fix.altitude
       << ",M,0.0,M,,";

    std::string sentence = ss.str();
    unsigned char checksum = 0;
    for (size_t i = 1; i < sentence.size(); ++i) {
      checksum ^= sentence[i];
    }
    char checksum_str[5];
    std::snprintf(checksum_str, sizeof(checksum_str), "*%02X", checksum);
    sentence += checksum_str;
    sentence += "\r\n";

    return sentence;
  }

  void streamRTCM()
  {
    while (!latest_fix_ && rclcpp::ok()) {
      rclcpp::sleep_for(500ms);
    }
    RCLCPP_INFO(this->get_logger(), "Received GPS fix. Starting NTRIP connection.");

    asio::ip::tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(host_, std::to_string(port_));
    asio::connect(socket_, endpoints);

    std::ostringstream request;
    request << "GET /" << mountpoint_ << " HTTP/1.1\r\n"
            << "Host: " << host_ << ":" << port_ << "\r\n"
            << "Ntrip-Version: Ntrip/2.0\r\n"
            << "User-Agent: NTRIP ROS2 Client\r\n"
            << "Authorization: Basic " << base64Encode(username_ + ":" + password_) << "\r\n"
            << "Connection: close\r\n\r\n";

    std::string gga = generateGGA();

    socket_.send(asio::buffer(request.str()));
    std::this_thread::sleep_for(500ms);
    socket_.send(asio::buffer(gga));

    std::vector<char> buffer(4096);
    while (rclcpp::ok()) {
      std::error_code ec;
      size_t len = socket_.read_some(asio::buffer(buffer), ec);
      if (ec) break;
      auto message = std::make_unique<rtcm_msgs::msg::Message>();
      message->header.stamp = this->now();
      message->header.frame_id = mountpoint_;
      message->message.assign(buffer.begin(), buffer.begin() + len);
      rtcm_pub_->publish(std::move(message));
    }
  }

  std::string base64Encode(const std::string & input)
  {
    static const std::string chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    int val = 0, valb = -6;
    for (uint8_t c : input) {
      val = (val << 8) + c;
      valb += 8;
      while (valb >= 0) {
        result.push_back(chars[(val >> valb) & 0x3F]);
        valb -= 6;
      }
    }
    if (valb > -6) result.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
    while (result.size() % 4) result.push_back('=');
    return result;
  }
};

}  // namespace ntrip_client

RCLCPP_COMPONENTS_REGISTER_NODE(ntrip_client::NTRIPClientNode)
