#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
// include usbcomms.cpp
#include "usbcomms.cpp"

// The two Pico boards now enumerate as USB-serial devices (USB-CDC
// "virtual UART") instead of talking over the old SPI bus, so the
// Jetson side needs to know which /dev node each one landed on. These
// are exposed as ROS parameters below (motor_usb_port / rpm_usb_port)
// so they can be set from a launch file or config yaml without a
// rebuild; the values here are just the fallback defaults.
constexpr const char* kDefaultMotorUsbPort = "/dev/ttyACM0";
constexpr const char* kDefaultRpmUsbPort = "/dev/ttyACM1";
// The Picos present themselves as USB-CDC virtual serial ports, which
// (unlike a real UART) transfer at full USB speed regardless of the
// requested baud rate - termios still wants a value, so this is kept
// for API compliance/portability rather than because it changes
// anything on the wire.
constexpr speed_t kUsbBaudRate = B115200;

class SPINode : public rclcpp::Node {
    public:
        SPINode() : Node("spi_node") {
            // Config variables for the USB ports the two Pico boards
            // (motor/steering driver and wheel-speed/RPM board) show
            // up on.
            this->declare_parameter<std::string>("motor_usb_port", kDefaultMotorUsbPort);
            this->declare_parameter<std::string>("rpm_usb_port", kDefaultRpmUsbPort);

            motor_usb_port_ = this->get_parameter("motor_usb_port").as_string();
            rpm_usb_port_ = this->get_parameter("rpm_usb_port").as_string();

            motor_serial_ = std::make_unique<UsbSerialDevice>(motor_usb_port_, kUsbBaudRate);
            RCLCPP_INFO(this->get_logger(), "Connected to motor board on %s", motor_usb_port_.c_str());

            try {
                rpm_serial_ = std::make_unique<UsbSerialDevice>(rpm_usb_port_, kUsbBaudRate);
                RCLCPP_INFO(this->get_logger(), "Connected to RPM board on %s", rpm_usb_port_.c_str());
                rpm_reader_running_ = true;
                rpm_reader_thread_ = std::thread(&SPINode::rpm_reader_loop, this);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(),
                    "Could not open RPM board on %s (%s) - continuing without wheel-speed feedback",
                    rpm_usb_port_.c_str(), e.what());
            }

            subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
                "ackDrive", 10, std::bind(&SPINode::handle_ackermann_update, this, std::placeholders::_1));
        }

        ~SPINode() override {
            rpm_reader_running_ = false;
            if (rpm_reader_thread_.joinable()) {
                rpm_reader_thread_.join();
            }
        }

    private:

        std::string motor_usb_port_;
        std::string rpm_usb_port_;
        std::unique_ptr<UsbSerialDevice> motor_serial_;
        std::unique_ptr<UsbSerialDevice> rpm_serial_;

        std::thread rpm_reader_thread_;
        std::atomic<bool> rpm_reader_running_{false};

        std::atomic<double> left_speed{0.0};
        std::atomic<double> right_speed{0.0};

    void handle_ackermann_update(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Received Ackermann Drive - Speed: '%f', Steering Angle: '%f'", msg->speed, msg->steering_angle);
        double speed = msg->speed;
        double steering_angle = msg->steering_angle;
        // convert speed and steering angle to a command frame for the motor board
        std::vector<uint8_t> tx_data;
        tx_data.reserve(1 + sizeof(speed) + sizeof(steering_angle) + 1);
        // total size: 1 + 8 + 8 + 1 = 18 bytes

        // Start byte (helps the receiver align to frames)
        tx_data.push_back(0xAA);

        auto append_double = [&tx_data](double value) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
            tx_data.insert(tx_data.end(), p, p + sizeof(value));
        };

        append_double(speed);
        append_double(steering_angle);

        // Simple XOR checksum over everything after the start byte
        uint8_t checksum = 0;
        for (std::size_t i = 1; i < 17; ++i) {
            checksum ^= tx_data[i];
        }
        tx_data.push_back(checksum);

        // --- Send the command frame to the motor board over USB serial ---
        // Unlike the old SPI link (a clocked, full-duplex bus that
        // always echoed a reply byte for every byte written), UART
        // has no built-in reply - the motor board just applies the
        // command and doesn't answer back. Wheel-speed feedback comes
        // separately from the RPM board (see rpm_reader_loop below).
        try {
            motor_serial_->write(tx_data);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "USB write to motor board failed: %s", e.what());
        }
    }

    // Runs on its own thread for as long as the RPM board's USB
    // connection is open: the board streams a fresh telemetry frame
    // whenever it has one, so this just keeps scanning for the start
    // byte, reading a frame, and updating left_speed/right_speed on a
    // valid checksum.
    //
    // Frame layout (18 bytes, matches the Pico side):
    // [0] = 0xAA (start byte)
    // [1..8] = left wheel speed (double, 8 bytes)
    // [9..16] = right wheel speed (double, 8 bytes)
    // [17] = XOR checksum of bytes [1..16]
    void rpm_reader_loop() {
        while (rpm_reader_running_) {
            try {
                uint8_t start_byte = rpm_serial_->readByte();
                if (start_byte != 0xAA) {
                    continue;
                }

                std::vector<uint8_t> frame = rpm_serial_->read(17);

                uint8_t checksum = 0;
                for (std::size_t i = 0; i < 16; ++i) {
                    checksum ^= frame[i];
                }

                if (checksum != frame[16]) {
                    RCLCPP_WARN(this->get_logger(), "RPM board checksum mismatch, dropping frame");
                    continue;
                }

                double l = 0.0;
                double r = 0.0;
                std::memcpy(&l, frame.data(), sizeof(double));
                std::memcpy(&r, frame.data() + 8, sizeof(double));
                left_speed = l;
                right_speed = r;

                if (l > 0 || r > 0) {
                    RCLCPP_INFO(this->get_logger(), "Left speed: %lf, Right speed: %lf", l, r);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "USB read from RPM board failed: %s", e.what());
                return;
            }
        }
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    std::cout << "Initialized USB serial device!\n";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPINode>());
    rclcpp::shutdown();
}
