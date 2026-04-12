// spi_example.cpp
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

class SpiDevice {
public:
    SpiDevice(const std::string& devicePath,
              uint32_t speedHz,
              uint8_t mode = SPI_MODE_0,
              uint8_t bitsPerWord = 8)
        : devicePath_(devicePath),
          speedHz_(speedHz),
          bitsPerWord_(bitsPerWord)
    {
        fd_ = ::open(devicePath_.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "Failed to open " + devicePath_);
        }

        // Configure mode, bits, speed
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) == -1)
            throw_errno("SPI_IOC_WR_MODE");

        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord_) == -1)
            throw_errno("SPI_IOC_WR_BITS_PER_WORD");

        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz_) == -1)
            throw_errno("SPI_IOC_WR_MAX_SPEED_HZ");
    }

    ~SpiDevice() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // Full-duplex transfer: tx.size() bytes out, same number back
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& tx) const {
        if (tx.empty()) return {};

        std::vector<uint8_t> rx(tx.size(), 0);

        struct spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<__u64>(tx.data());
        tr.rx_buf = reinterpret_cast<__u64>(rx.data());
        tr.len    = static_cast<__u32>(tx.size());
        tr.speed_hz = speedHz_;
        tr.bits_per_word = bitsPerWord_;
        tr.delay_usecs = 0;

        // go byte-by-byte and transfer
	
        // for (size_t i = 0; i < tx.size(); ++i) {
        //     tr.len = 1;
        //     tr.tx_buf = reinterpret_cast<__u64>(tx.data() + i);
        //     tr.rx_buf = reinterpret_cast<__u64>(rx.data() + i);
        //     int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
        //     if (ret < 1) {
        //         throw_errno("SPI_IOC_MESSAGE (byte transfer)");
        //     }
        // }
      tr.len = tx.size();
      ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);

        return rx;
    }

    // Write-only convenience
    void write(const std::vector<uint8_t>& tx) const {
        if (tx.empty()) return;

        struct spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<__u64>(tx.data());
        tr.len    = static_cast<__u32>(tx.size());
        tr.speed_hz = speedHz_;
        tr.bits_per_word = bitsPerWord_;

        int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) {
            throw_errno("SPI_IOC_MESSAGE (write)");
        }
    }

private:
    void throw_errno(const char* what) const {
        throw std::system_error(errno, std::generic_category(), what);
    }

    std::string devicePath_;
    int fd_{-1};
    uint32_t speedHz_;
    uint8_t bitsPerWord_;
};
