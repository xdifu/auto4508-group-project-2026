#include <algorithm>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lakibeam1/data_type.h"
#include "lakibeam1/remote.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr uint16_t kLakibeamDataFlag = 0xEEFF;

std::string to_sensor_bool(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class Lakibeam1ScanNode : public rclcpp::Node
{
public:
  Lakibeam1ScanNode()
  : Node(
      "lakibeam1_scan_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    frame_id_ = read_string_parameter("frame_id", "laser_frame");
    output_topic_ = read_string_parameter("output_topic", "scan");
    host_ip_ = read_string_parameter("hostip", "0.0.0.0");
    sensor_ip_ = read_string_parameter("sensorip", "192.168.198.2");
    inverted_ = read_bool_parameter("inverted", false);
    port_ = read_int_parameter("port", 2368);
    angle_offset_deg_ = read_int_parameter("angle_offset", 0);
    apply_sensor_config_ = read_bool_parameter("apply_sensor_config", false);
    scan_frequency_hz_ = read_int_parameter("scanfreq", 30);
    filter_level_ = read_int_parameter("filter", 3);
    laser_enable_ = read_bool_parameter("laser_enable", true);
    scan_range_start_deg_ = read_int_parameter("scan_range_start", 45);
    scan_range_stop_deg_ = read_int_parameter("scan_range_stop", 315);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Lakibeam driver configured: topic=%s frame=%s host_ip=%s sensor_ip=%s port=%d",
      output_topic_.c_str(), frame_id_.c_str(), host_ip_.c_str(), sensor_ip_.c_str(), port_);

    if (apply_sensor_config_) {
      apply_sensor_configuration();
    } else {
      RCLCPP_INFO(get_logger(), "Lakibeam sensor-side configuration updates disabled.");
    }

    create_socket();
    recv_thread_ = std::thread(&Lakibeam1ScanNode::scan_publish_loop, this);
  }

  ~Lakibeam1ScanNode() override
  {
    stop_requested_ = true;
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

private:
  rclcpp::ParameterValue read_parameter_value(const std::string & name) const
  {
    rclcpp::Parameter parameter;
    if (get_parameter(name, parameter)) {
      return parameter.get_parameter_value();
    }
    return rclcpp::ParameterValue{};
  }

  std::string read_string_parameter(const std::string & name, const std::string & default_value) const
  {
    const auto value = read_parameter_value(name);
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_STRING:
        return value.get<std::string>();
      default:
        throw std::runtime_error("Lakibeam parameter '" + name + "' must be a string value");
    }
  }

  int read_int_parameter(const std::string & name, const int default_value) const
  {
    const auto value = read_parameter_value(name);
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<int>(value.get<int64_t>());
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return static_cast<int>(value.get<double>());
      case rclcpp::ParameterType::PARAMETER_STRING:
        return std::stoi(value.get<std::string>());
      default:
        throw std::runtime_error("Lakibeam parameter '" + name + "' must be an integer-compatible value");
    }
  }

  bool read_bool_parameter(const std::string & name, const bool default_value) const
  {
    const auto value = read_parameter_value(name);
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return value.get<bool>();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return value.get<int64_t>() != 0;
      case rclcpp::ParameterType::PARAMETER_STRING: {
        auto text = value.get<std::string>();
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
          return static_cast<char>(std::tolower(ch));
        });
        if (text == "true" || text == "1" || text == "yes" || text == "on") {
          return true;
        }
        if (text == "false" || text == "0" || text == "no" || text == "off") {
          return false;
        }
        break;
      }
      default:
        break;
    }

    throw std::runtime_error("Lakibeam parameter '" + name + "' must be a bool-compatible value");
  }

  void apply_sensor_configuration()
  {
    const std::vector<std::pair<std::string, std::string>> config_requests = {
      {"/api/v1/sensor/scanfreq", std::to_string(scan_frequency_hz_)},
      {"/api/v1/sensor/laser_enable", to_sensor_bool(laser_enable_)},
      {"/api/v1/sensor/scan_range/start", std::to_string(scan_range_start_deg_)},
      {"/api/v1/sensor/scan_range/stop", std::to_string(scan_range_stop_deg_)}
    };

    for (const auto & [path, value] : config_requests) {
      if (!sensor_config(sensor_ip_, path, value)) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to apply Lakibeam sensor config %s=%s. Continuing with current device settings.",
          path.c_str(), value.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Lakibeam filter level is set to %d for operator visibility, but is not pushed over REST "
      "because the official driver does not document a stable filter endpoint.",
      filter_level_);
  }

  void create_socket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Failed to create Lakibeam UDP socket");
    }

    const int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (inet_aton(host_ip_.c_str(), &server_addr.sin_addr) == 0) {
      throw std::runtime_error("Invalid Lakibeam host IP: " + host_ip_);
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
      throw std::runtime_error(
              "Failed to bind Lakibeam UDP socket to " + host_ip_ + ":" + std::to_string(port_));
    }
  }

  void scan_publish_loop()
  {
    rclcpp::Time scan_begin = now();
    rclcpp::Time scan_end = scan_begin;
    int block_index = 12;
    int resolution = 25;
    std::vector<ScanPoint> scan_points;
    scan_points.reserve(2048);

    while (rclcpp::ok() && !stop_requested_) {
      if (block_index == 12) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        MsopDataPacket packet{};
        const ssize_t received = recvfrom(
          socket_fd_, &packet, sizeof(packet), 0,
          reinterpret_cast<sockaddr *>(&client_addr), &client_len);

        if (received < 0) {
          continue;
        }

        char sender_ip[INET_ADDRSTRLEN] = {0};
        if (inet_ntop(AF_INET, &client_addr.sin_addr, sender_ip, sizeof(sender_ip)) != nullptr) {
          if (!sensor_ip_.empty() && sensor_ip_ != sender_ip) {
            continue;
          }
          RCLCPP_INFO_ONCE(
            get_logger(),
            "Lakibeam UDP packets are arriving from %s:%u",
            sender_ip,
            static_cast<unsigned>(ntohs(client_addr.sin_port)));
        }

        if (packet.blocks[0].azimuth == 0) {
          scan_end = scan_begin;
          scan_begin = now();
        }

        const int azimuth_delta =
          static_cast<int>(packet.blocks[1].azimuth) - static_cast<int>(packet.blocks[0].azimuth);
        if (azimuth_delta > 0) {
          resolution = azimuth_delta / 16;
        }

        block_index = 0;

        for (; block_index < 12; ++block_index) {
          const auto & block = packet.blocks[block_index];
          if (block.data_flag != kLakibeamDataFlag) {
            continue;
          }

          for (int i = 0; i < 16; ++i) {
            const uint16_t angle = static_cast<uint16_t>(block.azimuth + resolution * i);
            if (angle == 0 && !scan_points.empty()) {
              publish_scan(scan_begin, scan_end, scan_points);
              scan_points.clear();
            }

            ScanPoint point{};
            point.angle = angle;
            point.distance_mm = block.results[i].dist_1;
            point.rssi = block.results[i].rssi_1;
            point.timestamp = packet.timestamp;
            scan_points.push_back(point);
          }
        }

        block_index = 12;
      }
    }
  }

  void publish_scan(
    const rclcpp::Time & scan_begin,
    const rclcpp::Time & scan_end,
    const std::vector<ScanPoint> & scan_points)
  {
    if (scan_points.empty()) {
      return;
    }

    sensor_msgs::msg::LaserScan scan_msg;
    const auto num_readings = scan_points.size();
    double duration = (scan_begin - scan_end).seconds();
    if (duration <= 0.0) {
      duration = 1.0 / std::max(1, scan_frequency_hz_);
    }

    // Stamp scans at publish time so downstream TF consumers do not see the data as stale when the
    // azimuth wrap happens mid-packet and the previous scan start time is older than the active TF cache.
    scan_msg.header.stamp = now();
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = static_cast<float>((-180.0 + angle_offset_deg_) * kDegToRad);
    scan_msg.angle_max = static_cast<float>((180.0 + angle_offset_deg_) * kDegToRad);
    scan_msg.angle_increment = static_cast<float>((2.0 * kPi) / static_cast<double>(num_readings));
    scan_msg.scan_time = static_cast<float>(duration);
    scan_msg.time_increment = static_cast<float>(duration / static_cast<double>(num_readings));
    scan_msg.range_min = 0.0F;
    scan_msg.range_max = 100.0F;
    scan_msg.ranges.assign(num_readings, std::numeric_limits<float>::infinity());
    scan_msg.intensities.assign(num_readings, 0.0F);

    for (size_t i = 0; i < num_readings; ++i) {
      const size_t target_index = inverted_ ? (num_readings - i - 1) : i;
      const float range_m = static_cast<float>(scan_points[i].distance_mm) / 1000.0F;
      if (range_m > 0.0F) {
        scan_msg.ranges[target_index] = range_m;
        scan_msg.intensities[target_index] = static_cast<float>(scan_points[i].rssi);
      }
    }

    scan_pub_->publish(scan_msg);
    RCLCPP_INFO_ONCE(
      get_logger(),
      "Published first Lakibeam LaserScan with %zu points and frame_id=%s",
      num_readings,
      frame_id_.c_str());
  }

  std::string frame_id_;
  std::string output_topic_;
  std::string host_ip_;
  std::string sensor_ip_;
  bool inverted_{false};
  bool apply_sensor_config_{false};
  bool laser_enable_{true};
  int port_{2368};
  int angle_offset_deg_{0};
  int scan_frequency_hz_{30};
  int filter_level_{3};
  int scan_range_start_deg_{45};
  int scan_range_stop_deg_{315};
  int socket_fd_{-1};
  std::atomic<bool> stop_requested_{false};
  std::thread recv_thread_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lakibeam1ScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
