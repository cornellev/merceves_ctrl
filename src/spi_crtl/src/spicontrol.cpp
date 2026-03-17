#include <iostream>
#include <vector>
#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
// include spicomms.cpp
#include "spicomms.cpp"

const std::string dev = "/dev/spidev0.1";
uint32_t speedHz = 500'000;  // 1 MHz

SpiDevice spi(dev, speedHz, SPI_MODE_0, 8);

class SPINode : public rclcpp::Node {
    public:
        SPINode() : Node("spi_node") {
            subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("ackDrive", 10, std::bind(&SPINode::handle_ackermann_update, this, std::placeholders::_1));
        }

    private:

	double left_speed, right_speed;

    void handle_ackermann_update(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Received Ackermann Drive - Speed: '%f', Steering Angle: '%f'", msg->speed, msg->steering_angle);
        double speed = msg->speed;
        double steering_angle = msg->steering_angle;
        // covnvert speed and steering angle to SPI data
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

        // RCLCPP_INFO(this->get_logger(), "SPI write successful, sent %zu bytes", tx_data.size());
        // RCLCPP_INFO(this->get_logger(), "Data bytes:");
        // for (size_t i = 0; i < tx_data.size(); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "Byte %zu: 0x%02X", i, tx_data[i]);
        // }

        // --- Actually send over SPI ---
	/*
	try {
	    for (size_t i = 0; i < tx_data.size(); i++) {
		std::vector<uint8_t> to_send = {tx_data[i]};
		spi.write(to_send);
	    }
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "SPI write failed: %s", e.what());
	}
	*/

	std::vector<uint8_t> rx_data = spi.transfer(tx_data);
        
        checksum:
	checksum = 0;
	for(std::size_t i = 1; i < rx_data.size() - 1; ++i) {
             checksum ^= rx_data[i];
	}

	if(checksum == rx_data[rx_data.size() - 1]) {
    	    uint64_t l_bytes = 0;
            uint64_t r_bytes = 0;

	    for(int i = 1; i < 9; i++) {
	        l_bytes |= (uint64_t)rx_data[i] << ((i-1)*8);
	    }

	    for(int i = 9; i < 17; i++) {
		r_bytes |= (uint64_t)rx_data[i] << ((i-9)*8);
	    }

            std::memcpy(&left_speed, &l_bytes, sizeof(left_speed));
	    std::memcpy(&right_speed, &r_bytes, sizeof(right_speed));

	} else {
	    RCLCPP_ERROR(this->get_logger(), "SPI checksum failed :(");
	    std::vector<uint8_t> header_byte;
	    header_byte.reserve(1);
	    header_byte.push_back(0xAA);

	    std::vector<uint8_t> tx_data_truncated(
		tx_data.begin() + 1,
		tx_data.begin() + 18
	    );

	    bool misaligned = true;
	    std::vector<uint8_t> other_data;
	    other_data.reserve(17);
	    while(misaligned) {
	        std::vector<uint8_t> header_back = spi.transfer(header_byte);
		if(header_back[0] == 0xAA) {
		    other_data = spi.transfer(tx_data_truncated);
		    misaligned = false;
		}
	    }
            rx_data[0] = 0xAA;
	    RCLCPP_INFO(this->get_logger(), "length of other_data: %ld", other_data.size());
	    for(int i = 0; i < 17; i++) {
                rx_data[i+1] = other_data[i];
	    }
	    goto checksum;
	}

	if(left_speed > 0 || right_speed > 0) {
		RCLCPP_INFO(this->get_logger(), "Left speed: %lf, Right speed: %lf", left_speed, right_speed);
	}
	// RCLCPP_INFO(this->get_logger(), "RX: %s", ([&]{ std::ostringstream s; for(uint8_t b : rx_data) s << std::hex << std::setw(2) << std::setfill('0') << (int)b << " "; return s.str(); })().c_str());
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    std::cout << "Initialized spi device!\n";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPINode>());
    rclcpp::shutdown();
}
