#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>
#include <cmath>
#include <chrono>
#include <thread>

struct ServoData
{
  float position[7];
  float velocity[7];
};

using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace hardware_controller 
{

class HardwareController : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Initialize clock for throttled logging
    clock_ = rclcpp::Clock::make_shared();

    // Log hardware parameters for debugging
    for (const auto &param : info_.hardware_parameters) {
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Parameter: %s = %s", 
                  param.first.c_str(), param.second.c_str());
    }

    // Initialize hardware parameters with defaults
    port_ = "/dev/ttyACM0";
    baudrate_ = 115200;
    num_servos_ = 8;

    // Initialize state and command vectors
    positions_.resize(num_servos_, 0.0);
    velocities_.resize(num_servos_, 0.0);
    position_commands_.resize(num_servos_, 0.0);
    velocity_commands_.resize(num_servos_, 0.0);
    
    // Initialize communication state
    retry_count_ = 0;
    communication_error_ = false;
    consecutive_errors_ = 0;
    last_successful_read_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Hardware controller initialized");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Opening serial port: %s", port_.c_str());
    
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                   "Failed to open serial port: %s | %s", port_.c_str(), strerror(errno));
      return CallbackReturn::ERROR;
    }

    // Configure serial port
    if (!configure_serial_port()) {
      close(fd_);
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), 
                "Serial port opened successfully: %d", fd_);

    // Wait for Arduino to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Reset communication state
    retry_count_ = 0;
    communication_error_ = false;
    consecutive_errors_ = 0;
    last_successful_read_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Hardware controller activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Hardware controller deactivated");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < num_servos_; ++i) {
      interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "position", &positions_[i]));

      interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "velocity", &velocities_[i]));
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() 
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < num_servos_; ++i) {
      interfaces.emplace_back(info_.joints[i].name, "position", &position_commands_[i]);
      interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_commands_[i]);
    }
    return interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) 
  {
    if (!started_) {
      return return_type::OK;
    }

    // Check if we should skip this read due to too many consecutive errors
    if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      auto now = std::chrono::steady_clock::now();
      auto time_since_last_success = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (now - last_successful_read_).count();
      
      if (time_since_last_success > ERROR_RECOVERY_TIMEOUT_MS) {
        RCLCPP_WARN(rclcpp::get_logger("HardwareController"), 
                   "Attempting to recover from communication errors");
        consecutive_errors_ = 0;
        communication_error_ = false;
      } else {
        // Use last known positions during error recovery
        return return_type::OK;
      }
    }

    // Send feedback request
    if (!send_feedback_request()) {
      handle_communication_error("Failed to send feedback request");
      return return_type::OK;
    }

    // Wait for Arduino response (adaptive timing)
    int wait_time_ms = std::min(BASE_WAIT_TIME_MS + (retry_count_ * 50), MAX_WAIT_TIME_MS);
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));

    // Read response
    if (!read_servo_feedback()) {
      handle_communication_error("Failed to read servo feedback");
      return return_type::OK;
    }

    // Reset error counters on successful read
    retry_count_ = 0;
    consecutive_errors_ = 0;
    communication_error_ = false;
    last_successful_read_ = std::chrono::steady_clock::now();

    // Log position data occasionally
    log_servo_data();

    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (!started_) {
      started_ = true;
    }

    // Skip write if we have communication errors
    if (communication_error_ && consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      return return_type::OK;
    }

    // Clear output buffer
    tcflush(fd_, TCOFLUSH);

    // Prepare servo data
    ServoData servo_data;
    for (size_t i = 0; i < 7; ++i) {
      double pos_deg = position_commands_[i] * (180.0 / M_PI);
      double vel_deg = velocity_commands_[i] * (180.0 / M_PI);
      
      // Clamp position to safe limits
      pos_deg = std::max(-90.0, std::min(90.0, pos_deg));
      
      servo_data.position[6-i] = static_cast<float>(pos_deg);
      servo_data.velocity[6-i] = 100.0f; // Fixed velocity for now
    }

    // Send move command
    const uint8_t *payload_ptr = reinterpret_cast<const uint8_t*>(&servo_data);
    std::vector<uint8_t> payload(payload_ptr, payload_ptr + sizeof(ServoData));

    constexpr uint8_t CMD_MOVE = 'M';
    if (!send_packet(CMD_MOVE, payload)) {
      handle_communication_error("Failed to send move command");
      return return_type::OK;
    }

    // Log command data occasionally
    log_command_data(servo_data);

    return return_type::OK;
  }

private:
  // Configuration constants
  static constexpr int MAX_RETRIES = 3;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 10;
  static constexpr int BASE_WAIT_TIME_MS = 150;
  static constexpr int MAX_WAIT_TIME_MS = 300;
  static constexpr int READ_TIMEOUT_MS = 250;
  static constexpr int ERROR_RECOVERY_TIMEOUT_MS = 5000;
  static constexpr size_t HEADER_SIZE = 1;
  static constexpr size_t PACKET_META_SIZE = 2 + 1 + 2;
  static constexpr size_t PAYLOAD_SIZE = sizeof(ServoData);
  static constexpr size_t FULL_PACKET_SIZE = HEADER_SIZE + PACKET_META_SIZE + PAYLOAD_SIZE + 1 + 1;

  // Hardware parameters
  std::string port_;
  int baudrate_;
  int fd_ = -1;
  size_t num_servos_;
  struct termios tty_;
  bool started_ = false;

  // State vectors
  std::vector<double> positions_, velocities_, position_commands_, velocity_commands_;

  // Communication state
  int retry_count_;
  int consecutive_errors_;
  bool communication_error_;
  std::chrono::steady_clock::time_point last_successful_read_;

  // Logging counters
  int read_log_counter_ = 0;
  int write_log_counter_ = 0;

  // Clock for throttled logging
  std::shared_ptr<rclcpp::Clock> clock_;

  bool configure_serial_port()
  {
    if (tcgetattr(fd_, &tty_) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                   "Error getting serial attributes: %s", strerror(errno));
      return false;
    }

    // Configure serial port settings
    tty_.c_cflag &= ~PARENB;    // No parity
    tty_.c_cflag &= ~CSTOPB;    // One stop bit
    tty_.c_cflag &= ~CSIZE;     // Clear data size bits
    tty_.c_cflag |= CS8;        // 8 data bits
    tty_.c_cflag &= ~CRTSCTS;   // No hardware flow control
    tty_.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem controls

    tty_.c_lflag &= ~ICANON;    // Non-canonical mode
    tty_.c_lflag &= ~ECHO;      // No echo
    tty_.c_lflag &= ~ECHOE;     // No erasure
    tty_.c_lflag &= ~ECHONL;    // No new-line echo
    tty_.c_lflag &= ~ISIG;      // No signal interpretation

    tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty_.c_oflag &= ~OPOST;     // No output processing
    tty_.c_oflag &= ~ONLCR;     // No CR to NL conversion

    // Timeout settings
    tty_.c_cc[VTIME] = 2;       // 0.2 second timeout
    tty_.c_cc[VMIN] = 0;        // Non-blocking read

    // Set baud rate
    speed_t speed = B115200;
    cfsetospeed(&tty_, speed);
    cfsetispeed(&tty_, speed);

    // Apply settings
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty_) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                   "Error setting serial attributes: %s", strerror(errno));
      return false;
    }

    return true;
  }

  int read_serial_with_timeout(unsigned char* buf, int nBytes)
  {
    auto start_time = std::chrono::steady_clock::now();
    int total_read = 0;
    int current_retry = 0;

    while (total_read < nBytes && current_retry < MAX_RETRIES) {
      int bytes_to_read = nBytes - total_read;
      int bytes_read = ::read(fd_, &buf[total_read], bytes_to_read);
      
      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // No data available, check timeout
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::steady_clock::now() - start_time).count();
          
          if (elapsed > READ_TIMEOUT_MS) {
            current_retry++;
            if (current_retry < MAX_RETRIES) {
              RCLCPP_WARN(rclcpp::get_logger("HardwareController"), 
                         "Read timeout, retry %d/%d", current_retry, MAX_RETRIES);
              tcflush(fd_, TCIOFLUSH);
              total_read = 0;
              start_time = std::chrono::steady_clock::now();
              continue;
            } else {
              RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                          "Read failed after %d retries", MAX_RETRIES);
              return -1;
            }
          }
          
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                      "Read error: %s", strerror(errno));
          return -1;
        }
      }
      
      if (bytes_read > 0) {
        total_read += bytes_read;
        start_time = std::chrono::steady_clock::now(); // Reset timeout on successful read
      }
    }

    return total_read;
  }

  uint8_t calculate_checksum(uint8_t command, uint16_t packet_size, uint16_t payload_size, 
                            const std::vector<uint8_t> &payload) 
  {
    uint32_t checksum = 0;
    checksum += command;
    checksum += packet_size & 0xFF;
    checksum += (packet_size >> 8) & 0xFF;
    checksum += payload_size & 0xFF;
    checksum += (payload_size >> 8) & 0xFF;
    for (uint8_t byte : payload) {
      checksum += byte;
    }
    return checksum & 0xFF;
  }

  bool send_packet(uint8_t command, const std::vector<uint8_t>& payload)
  {
    const char *STRUCT_HEADER = "S";
    uint16_t payload_size = static_cast<uint16_t>(payload.size());
    uint16_t packet_size = 1 + 2 + payload_size + 1 + 1; // command + payload_size + payload + checksum + '\n'

    uint8_t checksum = calculate_checksum(command, packet_size, payload_size, payload);

    std::vector<uint8_t> packet;
    packet.insert(packet.end(), STRUCT_HEADER, STRUCT_HEADER + 1);

    // Packet size (little endian)
    packet.push_back(packet_size & 0xFF);
    packet.push_back((packet_size >> 8) & 0xFF);

    // Command
    packet.push_back(command);

    // Payload size (little endian)
    packet.push_back(payload_size & 0xFF);
    packet.push_back((payload_size >> 8) & 0xFF);

    // Payload
    packet.insert(packet.end(), payload.begin(), payload.end());

    // Checksum
    packet.push_back(checksum);

    // End of command
    packet.push_back('\n');

    // Send packet in chunks to avoid buffer overflow
    size_t chunk_size = 60;
    ssize_t sent_total = 0;
    size_t packet_size_total = packet.size();

    while (sent_total < packet_size_total) {
      size_t chunk = std::min(chunk_size, packet_size_total - sent_total);
      ssize_t sent = ::write(fd_, packet.data() + sent_total, chunk);
      
      if (sent < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                    "Failed to send packet chunk: %s", strerror(errno));
        return false;
      }
      
      sent_total += sent;
      
      // Small delay between chunks
      if (sent_total < packet_size_total) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    return true;
  }

  bool send_feedback_request()
  {
    constexpr uint8_t CMD_FEEDBACK = 'F';
    std::vector<uint8_t> empty_payload;
    return send_packet(CMD_FEEDBACK, empty_payload);
  }

  bool read_servo_feedback()
  {
    std::vector<uint8_t> packet(FULL_PACKET_SIZE);

    ssize_t read_bytes = read_serial_with_timeout(packet.data(), FULL_PACKET_SIZE);
    tcflush(fd_, TCIFLUSH);

    if (read_bytes != static_cast<ssize_t>(FULL_PACKET_SIZE)) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Incomplete feedback packet: %ld/%ld bytes", read_bytes, FULL_PACKET_SIZE);
      return false;
    }

    // Validate packet structure
    if (!validate_packet(packet)) {
      return false;
    }

    // Extract servo data
    size_t offset = HEADER_SIZE + PACKET_META_SIZE;
    const uint8_t* payload_ptr = packet.data() + offset;
    const ServoData *data = reinterpret_cast<const ServoData *>(payload_ptr);

    // Update position and velocity states
    for (size_t i = 0; i < 7; ++i) {
      double pos_deg = static_cast<double>(data->position[i]);
      double vel_deg = static_cast<double>(data->velocity[i]);
      
      // Constrain position to valid range
      pos_deg = std::max(-90.0, std::min(90.0, pos_deg));
      
      double pos_rad = pos_deg * (M_PI / 180.0);
      double vel_rad = vel_deg * (M_PI / 180.0);

      positions_[i] = pos_rad;
      velocities_[i] = vel_rad;
    }

    return true;
  }

  bool validate_packet(const std::vector<uint8_t>& packet)
  {
    // Check header
    const char *STRUCT_HEADER = "S";
    if (std::memcmp(packet.data(), STRUCT_HEADER, 1) != 0) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Invalid header in feedback");
      return false;
    }

    size_t offset = 1;
    uint16_t packet_size = packet[offset] | (packet[offset + 1] << 8);
    offset += 2;

    uint8_t command = packet[offset++];
    if (command != 'F') {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Unexpected command in feedback: %02X", command);
      return false;
    }

    uint16_t payload_size = packet[offset] | (packet[offset + 1] << 8);
    offset += 2;

    if (payload_size != PAYLOAD_SIZE) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Payload size mismatch: expected %ld, got %d", PAYLOAD_SIZE, payload_size);
      return false;
    }

    // Validate end of command
    uint8_t eoc = packet[FULL_PACKET_SIZE - 1];
    if (eoc != '\n') {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Invalid end of command");
      return false;
    }

    // Validate checksum
    const uint8_t* payload_ptr = packet.data() + HEADER_SIZE + PACKET_META_SIZE;
    std::vector<uint8_t> payload(payload_ptr, payload_ptr + PAYLOAD_SIZE);
    uint8_t expected_checksum = calculate_checksum(command, packet_size, payload_size, payload);
    uint8_t received_checksum = packet[FULL_PACKET_SIZE - 2];

    if (expected_checksum != received_checksum) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HardwareController"), 
                           *clock_, 1000,
                           "Invalid checksum: expected %02X, got %02X", expected_checksum, received_checksum);
      return false;
    }

    return true;
  }

  void handle_communication_error(const std::string& error_msg)
  {
    consecutive_errors_++;
    communication_error_ = true;
    retry_count_++;

    if (consecutive_errors_ <= 3) {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), 
                 "%s (error %d)", error_msg.c_str(), consecutive_errors_);
    } else if (consecutive_errors_ == MAX_CONSECUTIVE_ERRORS) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), 
                  "Too many consecutive communication errors (%d), entering recovery mode", 
                  consecutive_errors_);
    }
  }

  void log_servo_data()
  {
    if (++read_log_counter_ % 100 == 0) { // Log every 100 reads (1 second at 100Hz)
      RCLCPP_DEBUG(rclcpp::get_logger("HardwareController"), 
                  "Positions: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                  positions_[0] * 180.0/M_PI, positions_[1] * 180.0/M_PI, 
                  positions_[2] * 180.0/M_PI, positions_[3] * 180.0/M_PI,
                  positions_[4] * 180.0/M_PI, positions_[5] * 180.0/M_PI, 
                  positions_[6] * 180.0/M_PI);
    }
  }

  void log_command_data(const ServoData& servo_data)
  {
    if (++write_log_counter_ % 100 == 0) { // Log every 100 writes (1 second at 100Hz)
      RCLCPP_DEBUG(rclcpp::get_logger("HardwareController"), 
                  "Commands: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                  servo_data.position[0], servo_data.position[1], servo_data.position[2],
                  servo_data.position[3], servo_data.position[4], servo_data.position[5], 
                  servo_data.position[6]);
    }
  }
};

} // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::HardwareController, hardware_interface::SystemInterface)