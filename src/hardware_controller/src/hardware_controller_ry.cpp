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

    for (const auto &param : info_.hardware_parameters) {
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Parameter: %s = %s", param.first.c_str(), param.second.c_str());
    }

    port_ = "/dev/ttyACM0";//info_.hardware_parameters["port"];
    baudrate_ = 115200;//std::stoi(info_.hardware_parameters["baudrate"]);
    num_servos_ = 8;//std::stoi(info_.hardware_parameters["num_servos"]);

    positions_.resize(num_servos_, 0.0);
    velocities_.resize(num_servos_, 0.0);

    position_commands_.resize(num_servos_, 0.0);
    velocity_commands_.resize(num_servos_, 0.0);
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Open Serial Port: %s", port_.c_str());
    fd_ = open(port_.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to open serial port: %s | %s", port_.c_str(), strerror(errno));
      return CallbackReturn::ERROR;
    }

    if (tcgetattr(fd_, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_controller_interface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(fd_);
        return CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0; // Read at least 1 character, timeout if not received in 1 decisecond

    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CustomHardware"), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), "SERIAL PORT OPENED: %d! WAITING...", fd_);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    // send_feedback_request();
    // char buffer[39];
    // std::string sync_sequence;
    // const std::string target = "STRT";

    // while (rclcpp::ok()) {
    //   int num_bytes = ::read(fd_, buffer, 1);
    //   if (num_bytes > 0) {
    //     sync_sequence += buffer[0];
    //     // Keep only the last 4 characters
    //     if (sync_sequence.length() > 4) {
    //       sync_sequence.erase(0, sync_sequence.length() - 4);
    //     }
    //     if (sync_sequence == target) {
    //       RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Synchronization string 'STRT' received.");
    //       break;
    //     }
    //   } else {
    //     RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "No data yet from serial, waiting...");
    //   }
    // }


    // ::read(fd_, buffer, 35);

    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Activated HardwareController");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (fd_ >= 0) {
      close(fd_);
    }
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
      std::cout << info_.joints[i].name << "\n";
      interfaces.emplace_back(info_.joints[i].name, "position", &position_commands_[i]);

      interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_commands_[i]);
    }
    return interfaces;
  }

  int ReadSerial(unsigned char* buf, int nBytes)
  {
      auto t_start = std::chrono::high_resolution_clock::now();
      int n = 0;
      while(n < nBytes)
      {
          int ret = ::read(fd_, &buf[n], 1);
          if(ret < 0) return ret;
  
          n+=ret;
          auto t_end = std::chrono::high_resolution_clock::now();
          double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
          if(elapsed_time_ms > 100)
          {
              RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "RETRY READ");
              send_feedback_request();
              std::this_thread::sleep_for(std::chrono::milliseconds(10));

              n = 0;
              t_start = std::chrono::high_resolution_clock::now();
          }
      }
      return n;
  }

  uint8_t calculate_checksum(uint8_t command, uint16_t packet_size, uint16_t payload_size, const std::vector<uint8_t> &payload) {
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
    packet.insert(packet.end(), STRUCT_HEADER, STRUCT_HEADER + 1); // Header

    // Packet size
    packet.push_back(packet_size & 0xFF);
    packet.push_back((packet_size >> 8) & 0xFF);

    // Command
    packet.push_back(command);

    // Payload size
    packet.push_back(payload_size & 0xFF);
    packet.push_back((payload_size >> 8) & 0xFF);

    // Payload
    packet.insert(packet.end(), payload.begin(), payload.end());

    // Checksum
    packet.push_back(checksum);

    // End of command
    packet.push_back('\n');

    // Send over serial in chunks
    size_t chunk_size = 60; // Adjust depending on your system's buffer size
    ssize_t sent_total = 0;
    size_t packet_size_total = packet.size();

    while (sent_total < packet_size_total)
    {
        size_t chunk = std::min(chunk_size, packet_size_total - sent_total);
        ssize_t sent = ::write(fd_, packet.data() + sent_total, chunk);
        if (sent < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to send packet chunk: %s", strerror(errno));
            return false;
        }

        sent_total += sent;
    }

    return true;
  }

  bool send_feedback_request()
  {
    constexpr uint8_t CMD_FEEDBACK = 'F';
    std::vector<uint8_t> empty_payload;
    return send_packet(CMD_FEEDBACK, empty_payload);
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) {
    if (!started) {
      return return_type::OK;
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "STARTED READ");
    bool result = send_feedback_request();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "SENT FEEDBACK REQ :%d", result);

    constexpr size_t HEADER_SIZE = 1;
    constexpr size_t PACKET_META_SIZE = 2 + 1 + 2; // packet_size + command + payload_size
    constexpr size_t PAYLOAD_SIZE = sizeof(ServoData); // 28 bytes
    constexpr size_t FULL_PACKET_SIZE = HEADER_SIZE + PACKET_META_SIZE + PAYLOAD_SIZE + 1 + 1; // + checksum + '\n'
  
    std::vector<uint8_t> packet(FULL_PACKET_SIZE);
  
    ssize_t read_bytes = ReadSerial(packet.data(), FULL_PACKET_SIZE);
    // ssize_t read_bytes = ::read(fd_, packet.data(), FULL_PACKET_SIZE);
    tcflush(fd_, TCIFLUSH);
    if (read_bytes != static_cast<ssize_t>(FULL_PACKET_SIZE)) {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Incomplete feedback packet: %ld/%ld bytes", read_bytes, FULL_PACKET_SIZE);
      return return_type::OK; // Ignore incomplete feedback
    }
  
    // Parse and validate
    const char *STRUCT_HEADER = "S";
    if (std::memcmp(packet.data(), STRUCT_HEADER, 1) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Invalid header in feedback %4s", packet.data());
      return return_type::OK;
    }
  
    size_t offset = 1;
    uint16_t packet_size = packet[offset] | (packet[offset + 1] << 8);
    offset += 2;
  
    uint8_t command = packet[offset++];
    if (command != 'F') {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Unexpected command in feedback %02X", command);
      return return_type::OK;
    }
  
    uint16_t payload_size = packet[offset] | (packet[offset + 1] << 8);
    offset += 2;
  
    if (payload_size != PAYLOAD_SIZE) {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Payload size mismatch: expected %ld, got %d", PAYLOAD_SIZE, payload_size);
      return return_type::OK;
    }
  
    const uint8_t* payload_ptr = packet.data() + offset;
    offset += PAYLOAD_SIZE;
  
    uint8_t received_checksum = packet[offset++];
    uint8_t eoc = packet[offset];
  
    if (eoc != '\n') {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Invalid end of command");
      return return_type::OK;
    }
  
    // Checksum validation
    std::vector<uint8_t> payload(payload_ptr, payload_ptr + PAYLOAD_SIZE);
    uint8_t expected_checksum = calculate_checksum(command, packet_size, payload_size, payload);
    if (expected_checksum != received_checksum) {
      RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Invalid checksum: expected %02X, got %02X", expected_checksum, received_checksum);
      return return_type::OK;
    }
  
    // Convert payload to struct
    const ServoData *data = reinterpret_cast<const ServoData *>(payload_ptr);
    for (size_t i = 0; i < 7; ++i) {
      int16_t constrained_position = std::max<int16_t>(-90, std::min<int16_t>(90, data->position[i]));
  
      double pos_rad = static_cast<double>(constrained_position) * (M_PI / 180.0);
      double vel_rad = static_cast<double>(data->velocity[i]) * (M_PI / 180.0);

      // if (true) {
      //   pos_rad = 0.0;
      //   vel_rad = 0.0;
      // }
  
      positions_[i] = pos_rad;
      velocities_[i] = vel_rad;
    }
  
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Got positions: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
          data->position[0], data->position[1], data->position[2],
          data->position[3], data->position[4], data->position[5], data->position[6]);
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Got velocities: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
          data->velocity[0], data->velocity[1], data->velocity[2],
          data->velocity[3], data->velocity[4], data->velocity[5], data->velocity[6]);
  
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    tcflush(fd_, TCOFLUSH);
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "STARTED WRITE");
    if (!started) {
      started = true;
      // return return_type::OK;
    }
  
    ServoData servo_data;
    for (size_t i = 0; i < 7; ++i) {
      double pos_deg = position_commands_[i] * (180.0 / M_PI);
      double vel_deg = velocity_commands_[i] * (180.0 / M_PI);
      servo_data.position[6-i] = static_cast<int16_t>(std::round(pos_deg));
      servo_data.velocity[6-i] = 100;//static_cast<int16_t>(std::round(vel_deg));
    }
  
    const uint8_t *payload_ptr = reinterpret_cast<const uint8_t*>(&servo_data);
    std::vector<uint8_t> payload(payload_ptr, payload_ptr + sizeof(ServoData));
  
    constexpr uint8_t CMD_MOVE = 'M';
    send_packet(CMD_MOVE, payload);
  
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "--------------------------------------------------------------");
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Sent positions: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
          servo_data.position[0], servo_data.position[1], servo_data.position[2],
          servo_data.position[3], servo_data.position[4], servo_data.position[5], servo_data.position[6]);
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Sent velocities: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
          servo_data.velocity[0], servo_data.velocity[1], servo_data.velocity[2],
          servo_data.velocity[3], servo_data.velocity[4], servo_data.velocity[5], servo_data.velocity[6]);
  
    

    // Send feedback request
    
    return return_type::OK;
  }

private:
  std::string port_;
  int baudrate_;
  int fd_ = -1;
  size_t num_servos_;
  struct termios tty;
  bool started = false;

  std::vector<double> positions_, velocities_, position_commands_, velocity_commands_;
};
};

PLUGINLIB_EXPORT_CLASS(hardware_controller::HardwareController, hardware_interface::SystemInterface)