// usbcomms.cpp
//
// Serial link to a Pico over its native USB port (USB-CDC "virtual
// UART"). This replaces the old spidev-based SpiDevice: the Jetson
// side of the link used to be a Linux spidev device (/dev/spidev0.1);
// now each Pico shows up as a plain USB-serial device node
// (e.g. /dev/ttyACM0) and we talk to it with a normal termios serial
// connection instead of ioctl(SPI_IOC_MESSAGE, ...).
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class UsbSerialDevice {
public:
    UsbSerialDevice(const std::string& devicePath, speed_t baudRate = B115200)
        : devicePath_(devicePath)
    {
        fd_ = ::open(devicePath_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "Failed to open " + devicePath_);
        }

        termios tty{};
        if (::tcgetattr(fd_, &tty) != 0) {
            throw_errno("tcgetattr");
        }

        // Raw byte-oriented mode: no line editing, no echo, no
        // CR/LF translation - we're moving a binary framed protocol,
        // not text.
        ::cfmakeraw(&tty);

        ::cfsetispeed(&tty, baudRate);
        ::cfsetospeed(&tty, baudRate);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSTOPB;   // 1 stop bit
        tty.c_cflag &= ~CRTSCTS; // no hardware flow control
        tty.c_cflag &= ~PARENB;  // no parity

        // Block until at least 1 byte is available, no extra
        // inter-byte timeout - reads below add their own framing.
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;

        if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw_errno("tcsetattr");
        }

        ::tcflush(fd_, TCIOFLUSH);
    }

    ~UsbSerialDevice() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    UsbSerialDevice(const UsbSerialDevice&) = delete;
    UsbSerialDevice& operator=(const UsbSerialDevice&) = delete;

    // Write-only convenience
    void write(const std::vector<uint8_t>& tx) const {
        if (tx.empty()) return;

        std::size_t written = 0;
        while (written < tx.size()) {
            ssize_t n = ::write(fd_, tx.data() + written, tx.size() - written);
            if (n < 0) {
                throw_errno("write");
            }
            written += static_cast<std::size_t>(n);
        }
    }

    // Blocking read of exactly `count` bytes.
    std::vector<uint8_t> read(std::size_t count) const {
        std::vector<uint8_t> rx(count, 0);
        std::size_t received = 0;
        while (received < count) {
            ssize_t n = ::read(fd_, rx.data() + received, count - received);
            if (n < 0) {
                throw_errno("read");
            }
            received += static_cast<std::size_t>(n);
        }
        return rx;
    }

    // Blocking read of a single byte - handy for scanning for a
    // framing/start byte.
    uint8_t readByte() const {
        return read(1)[0];
    }

    // Write tx, then block for a reply of the same length. UART isn't
    // clocked like SPI, so this is really "write, then wait for the
    // peer to answer" rather than a true simultaneous transfer - it's
    // provided for peers (like the motor board) that reply in-kind to
    // every command frame.
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& tx) const {
        write(tx);
        return read(tx.size());
    }

private:
    void throw_errno(const char* what) const {
        throw std::system_error(errno, std::generic_category(), what);
    }

    std::string devicePath_;
    int fd_{-1};
};
