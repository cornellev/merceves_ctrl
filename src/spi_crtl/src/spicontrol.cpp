#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
// include spicomms.cpp
#include "spicomms.cpp"

const std::string dev = "/dev/spidev0.1";
uint32_t speedHz = 500'000;  // 500 kHz

SpiDevice spi(dev, speedHz, SPI_MODE_0, 8);

class SPINode : public rclcpp::Node {
public:
    SPINode() : Node("spi_node") {
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "ackDrive", 10,
            std::bind(&SPINode::handle_ackermann_update, this, std::placeholders::_1));

        // 100 Hz timer drives SPI transfers, matching the Kalman filter rate.
        // Decoupled from ROS messages so RPM is always polled steadily.
        spi_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz
            std::bind(&SPINode::do_spi_transfer, this));
    }

private:
    // Cached command values (updated by ROS callback, read by timer)
    std::mutex cmd_mutex_;
    double cached_speed_ = 0.0;
    double cached_steering_ = 0.0;

    double left_speed_ = 0.0;
    double right_speed_ = 0.0;

    void handle_ackermann_update(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cached_speed_ = msg->speed;
        cached_steering_ = msg->steering_angle;
    }

    void do_spi_transfer() {
        // Snapshot the latest command under the lock
        double speed, steering_angle;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            speed = cached_speed_;
            steering_angle = cached_steering_;
        }

        // Build 18-byte TX frame
        std::vector<uint8_t> tx_data;
        tx_data.reserve(18);
        tx_data.push_back(0xAA);

        auto append_double = [&tx_data](double value) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
            tx_data.insert(tx_data.end(), p, p + sizeof(value));
        };

        append_double(speed);
        append_double(steering_angle);

        uint8_t checksum = 0;
        for (std::size_t i = 1; i < 17; ++i) {
            checksum ^= tx_data[i];
        }
        tx_data.push_back(checksum);

        // Single atomic 18-byte full-duplex transfer
        std::vector<uint8_t> rx_data = spi.transfer(tx_data);

        // Validate start byte
        if (rx_data[0] != 0xAA) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "SPI frame misaligned (got 0x%02X, expected 0xAA), skipping", rx_data[0]);
            return;
        }

        // Validate checksum over payload bytes [1..16]
        checksum = 0;
        for (std::size_t i = 1; i < 17; ++i) {
            checksum ^= rx_data[i];
        }

        if (checksum != rx_data[17]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "SPI checksum mismatch, skipping frame");
            return;
        }

        // Unpack RPM doubles from response
        std::memcpy(&left_speed_, &rx_data[1], sizeof(double));
        std::memcpy(&right_speed_, &rx_data[9], sizeof(double));

        if (left_speed_ > 0 || right_speed_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Left RPM: %lf, Right RPM: %lf",
                left_speed_, right_speed_);
        }
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr spi_timer_;
};

int main(int argc, char * argv[]) {
    std::cout << "Initialized spi device!\n";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPINode>());
    rclcpp::shutdown();
}
