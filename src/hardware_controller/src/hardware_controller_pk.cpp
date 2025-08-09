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

struct ServoInterface
{
  float positions[7];
  float velocities[7];
};

using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace hardware_controller
{

class HardwareController : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    port_ = "/dev/ttyACM0";
    baudrate_ = 57600;
    num_servos_ = 8;

    positions_.resize(num_servos_, 0.0);
    velocities_.resize(num_servos_, 0.0);

    position_commands_.resize(num_servos_, 0.0);
    velocity_commands_.resize(num_servos_, 0.0);

    directions_= {1, -1, -1, -1, -1, -1, 1};

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Opening Serial Port: %s", port_.c_str());
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to open serial port: %s | %s", port_.c_str(), strerror(errno));
        return CallbackReturn::ERROR;
    }

    if (tcgetattr(fd_, &tty_) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(fd_);
        return CallbackReturn::ERROR;
    }

    tty_.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty_.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty_.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty_.c_cflag |= CS8; // 8 bits per byte (most common)
    tty_.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty_.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO; // Disable echo
    tty_.c_lflag &= ~ECHOE; // Disable erasure
    tty_.c_lflag &= ~ECHONL; // Disable new-line echo
    tty_.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  
    tty_.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty_.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


    tty_.c_cc[VTIME] = 5; // Timeout in deciseconds (0.1s)
    tty_.c_cc[VMIN] = 0;  // Minimum number of characters to read

    cfsetospeed(&tty_, B57600);
    cfsetispeed(&tty_, B57600);

    tcflush(fd_, TCIFLUSH); // Flush input and output buffers
    if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error %i from tcsetattr: %s", errno, strerror(errno));
        close(fd_);
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Serial Port Opened");
    return CallbackReturn::SUCCESS;
}
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (fd_ >= 0)
    {
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

  return_type read(const rclcpp::Time &, const rclcpp::Duration &)
  {
      const uint8_t START_BYTE = 0xCC;
      const uint8_t END_BYTE = 0xDD;
  
      ServoInterface feedback;
      uint8_t buffer[sizeof(ServoInterface) + 2]; // Feedback + start + end bytes
      size_t total_bytes_read = 0;
  
      // Read the full packet
      while (total_bytes_read < sizeof(buffer))
      {
          ssize_t bytes_read = ::read(fd_, buffer + total_bytes_read, sizeof(buffer) - total_bytes_read);
          tcflush(fd_, TCIFLUSH);
          if (bytes_read < 0)
          {
              RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error reading from serial port: %s", strerror(errno));
              // std::fill(positions_.begin(), positions_.end(), 0.0);
              // std::fill(velocities_.begin(), velocities_.end(), 0.0);
              return return_type::ERROR;
          }
          total_bytes_read += bytes_read;

          // Add a small delay to avoid busy looping
         
      }
  
      // Validate start and end bytes
      if (buffer[0] != START_BYTE || buffer[sizeof(buffer) - 1] != END_BYTE)
      {
          RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Invalid feedback packet format");
          return return_type::OK;
      }
  
      // Extract the feedback structure
      std::memcpy(&feedback, buffer + 1, sizeof(ServoInterface));
  
      // Update positions and velocities
      for (size_t i = 0; i < num_servos_; ++i)
      {
          positions_[i] = (feedback.positions[i]* directions_[i]) * (M_PI / 180.0);
          velocities_[i] = (feedback.velocities[i]* directions_[i]) * (M_PI / 180.0);
      }
  
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Received positions: %f %f %f %f %f %f %f",
                  feedback.positions[0], feedback.positions[1], feedback.positions[2],
                  feedback.positions[3], feedback.positions[4], feedback.positions[5], feedback.positions[6]);
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Received velocities: %f %f %f %f %f %f %f",
                  feedback.velocities[0], feedback.velocities[1], feedback.velocities[2],
                  feedback.velocities[3], feedback.velocities[4], feedback.velocities[5], feedback.velocities[6]);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    const uint8_t START_BYTE = 0xAA;
    const uint8_t END_BYTE = 0xBB;

    ServoInterface command;
    // Convert input from radians to degrees per second
    for (size_t i = 0; i < num_servos_; ++i)
    {
      command.positions[i] = position_commands_[i] *(180.0 / M_PI)* directions_[i];
      command.velocities[i] = velocity_commands_[i] *(180.0 / M_PI)* directions_[i];
    }

    uint8_t buffer[sizeof(ServoInterface) + 2];
    buffer[0] = START_BYTE;
    std::memcpy(buffer + 1, &command, sizeof(ServoInterface));
    buffer[sizeof(buffer) - 1] = END_BYTE;

    ssize_t written_bytes = ::write(fd_, buffer, sizeof(buffer));
    if (written_bytes != sizeof(buffer))
    {
      RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to send command");
      return return_type::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Sent target velocities: %f %f %f %f %f %f %f",
          command.velocities[0], command.velocities[1], command.velocities[2],
          command.velocities[3], command.velocities[4], command.velocities[5], command.velocities[6]);
    RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Sent target positions: %f %f %f %f %f %f %f",
                command.positions[0], command.positions[1], command.positions[2],
                command.positions[3], command.positions[4], command.positions[5], command.positions[6]);

    return return_type::OK;
  }

private:
  std::string port_;
  int baudrate_;
  int fd_ = -1;
  size_t num_servos_;
  struct termios tty_;
  std::vector<int> directions_;

  std::vector<double> positions_, velocities_, position_commands_, velocity_commands_;
};

}; // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::HardwareController, hardware_interface::SystemInterface)