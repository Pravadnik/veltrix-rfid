#include "transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace rfid {

// ---------- TCP ----------

void TcpTransport::open() {
    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = nullptr;
    std::string port_s = std::to_string(port_);
    int gai = ::getaddrinfo(host_.c_str(), port_s.c_str(), &hints, &res);
    if (gai != 0) {
        throw TransportClosed("getaddrinfo(" + host_ + "): " + gai_strerror(gai));
    }

    int fd = -1;
    std::string last_err = "no address";
    for (struct addrinfo* p = res; p; p = p->ai_next) {
        fd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd < 0) { last_err = std::strerror(errno); continue; }

        // Connect with a timeout via temporary non-blocking mode.
        int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        int rc = ::connect(fd, p->ai_addr, p->ai_addrlen);
        if (rc < 0 && errno == EINPROGRESS) {
            fd_set wset;
            FD_ZERO(&wset);
            FD_SET(fd, &wset);
            struct timeval tv;
            tv.tv_sec = static_cast<long>(connect_timeout_);
            tv.tv_usec = static_cast<long>((connect_timeout_ - tv.tv_sec) * 1e6);
            rc = ::select(fd + 1, nullptr, &wset, nullptr, &tv);
            if (rc > 0) {
                int soerr = 0;
                socklen_t len = sizeof(soerr);
                ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len);
                rc = soerr ? -1 : 0;
                if (soerr) last_err = std::strerror(soerr);
            } else {
                last_err = (rc == 0) ? "connect timed out" : std::strerror(errno);
                rc = -1;
            }
        } else if (rc < 0) {
            last_err = std::strerror(errno);
        }
        if (rc == 0) {
            ::fcntl(fd, F_SETFL, flags);  // back to blocking
            break;
        }
        ::close(fd);
        fd = -1;
    }
    ::freeaddrinfo(res);
    if (fd < 0) {
        throw TransportClosed("connect " + host_ + ":" + port_s + ": " + last_err);
    }
    fd_ = fd;
}

void TcpTransport::close() {
    if (fd_ >= 0) {
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
}

void TcpTransport::write(const Bytes& data) {
    if (fd_ < 0) throw TransportClosed("socket not open");
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = ::send(fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) throw Interrupted();
            throw TransportClosed(std::string("send: ") + std::strerror(errno));
        }
        sent += static_cast<size_t>(n);
    }
}

void TcpTransport::set_read_timeout(double seconds) {
    if (fd_ < 0) return;
    struct timeval tv;
    if (seconds <= 0) {
        tv.tv_sec = 0;  // 0 = block indefinitely
        tv.tv_usec = 0;
    } else {
        tv.tv_sec = static_cast<long>(seconds);
        tv.tv_usec = static_cast<long>((seconds - tv.tv_sec) * 1e6);
    }
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

Bytes TcpTransport::read_exact(size_t n) {
    if (n == 0) return {};
    if (fd_ < 0) throw TransportClosed("socket not open");
    Bytes out;
    out.reserve(n);
    uint8_t buf[4096];
    while (out.size() < n) {
        size_t want = std::min(n - out.size(), sizeof(buf));
        ssize_t r = ::recv(fd_, buf, want, 0);
        if (r == 0) throw TransportClosed("connection closed by peer");
        if (r < 0) {
            if (errno == EINTR) throw Interrupted();
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // SO_RCVTIMEO fired. Only a timeout at a frame boundary means
                // "no reply"; a partial frame means the peer went silent mid-frame.
                if (out.empty()) throw Timeout("read timed out");
                throw TransportClosed("peer went silent mid-frame (read timeout)");
            }
            throw TransportClosed(std::string("recv: ") + std::strerror(errno));
        }
        out.insert(out.end(), buf, buf + r);
    }
    return out;
}

// ---------- Serial ----------

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return 0;
    }
}

void SerialTransport::open() {
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        throw TransportClosed("open " + port_ + ": " + std::strerror(errno));
    }
    struct termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        throw TransportClosed(std::string("tcgetattr: ") + std::strerror(errno));
    }
    speed_t sp = baud_to_speed(baudrate_);
    if (sp == 0) {
        ::close(fd);
        throw TransportClosed("unsupported baud rate: " + std::to_string(baudrate_));
    }
    ::cfsetispeed(&tty, sp);
    ::cfsetospeed(&tty, sp);

    // 8N1, raw mode, no flow control.
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | ISTRIP);
    tty.c_oflag &= ~OPOST;
    // Blocking read: return as soon as >=1 byte is available, no inter-byte timeout.
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        throw TransportClosed(std::string("tcsetattr: ") + std::strerror(errno));
    }
    ::tcflush(fd, TCIOFLUSH);
    fd_ = fd;
}

void SerialTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialTransport::write(const Bytes& data) {
    if (fd_ < 0) throw TransportClosed("serial port not open");
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
        if (n < 0) {
            if (errno == EINTR) throw Interrupted();
            throw TransportClosed(std::string("write: ") + std::strerror(errno));
        }
        sent += static_cast<size_t>(n);
    }
}

void SerialTransport::set_read_timeout(double seconds) {
    if (fd_ < 0) return;
    struct termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) return;
    if (seconds <= 0) {
        // Blocking: return as soon as >=1 byte is available, no timer.
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
        read_timeout_set_ = false;
    } else {
        // VMIN=0 makes VTIME a total read timer in tenths of a second.
        int deciseconds = static_cast<int>(seconds * 10);
        if (deciseconds < 1) deciseconds = 1;
        if (deciseconds > 255) deciseconds = 255;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = static_cast<cc_t>(deciseconds);
        read_timeout_set_ = true;
    }
    ::tcsetattr(fd_, TCSANOW, &tty);
}

Bytes SerialTransport::read_exact(size_t n) {
    if (n == 0) return {};
    if (fd_ < 0) throw TransportClosed("serial port not open");
    Bytes out;
    out.reserve(n);
    uint8_t buf[512];
    while (out.size() < n) {
        size_t want = std::min(n - out.size(), sizeof(buf));
        ssize_t r = ::read(fd_, buf, want);
        if (r == 0) {
            // With a timeout configured (VMIN=0), a 0-byte read means the timer
            // expired; otherwise (VMIN=1) it is a genuine EOF/port loss.
            if (read_timeout_set_) {
                if (out.empty()) throw Timeout("read timed out");
                throw TransportClosed("port went silent mid-frame (read timeout)");
            }
            throw TransportClosed("serial port returned EOF");
        }
        if (r < 0) {
            if (errno == EINTR) throw Interrupted();
            throw TransportClosed(std::string("read: ") + std::strerror(errno));
        }
        out.insert(out.end(), buf, buf + r);
    }
    return out;
}

// ---------- endpoint parsing ----------

std::pair<std::string, int> parse_endpoint(const std::string& spec, const std::string& kind) {
    if (kind == "tcp") {
        auto pos = spec.rfind(':');
        if (pos == std::string::npos || pos == 0 || pos + 1 >= spec.size()) {
            throw std::invalid_argument("expected HOST:PORT, got '" + spec + "'");
        }
        std::string host = spec.substr(0, pos);
        std::string port = spec.substr(pos + 1);
        for (char c : port) {
            if (!std::isdigit(static_cast<unsigned char>(c)))
                throw std::invalid_argument("expected HOST:PORT, got '" + spec + "'");
        }
        return {host, std::stoi(port)};
    }
    if (kind == "serial") {
        auto pos = spec.rfind(':');
        // Bare device path (or Windows COM6 with no baud) -> default baud.
        if (pos == std::string::npos) return {spec, 115200};
        std::string port = spec.substr(0, pos);
        std::string baud = spec.substr(pos + 1);
        for (char c : baud) {
            if (!std::isdigit(static_cast<unsigned char>(c)))
                throw std::invalid_argument("expected PORT[:BAUD], got '" + spec + "'");
        }
        if (baud.empty()) return {spec, 115200};
        return {port, std::stoi(baud)};
    }
    throw std::invalid_argument("unknown transport kind: " + kind);
}

}  // namespace rfid
