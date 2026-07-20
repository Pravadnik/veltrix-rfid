// Byte-level transports for the reader: TCP (network port) or serial (RS232).
//
// Both expose the same tiny interface the client needs: open(), close(),
// write(bytes), and read_exact(n) -> bytes (blocks until n bytes are available
// or the connection is gone). C++17 port of ../reader/transport.py.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "protocol.hpp"

namespace rfid {

class TransportClosed : public std::runtime_error {
public:
    explicit TransportClosed(const std::string& m) : std::runtime_error(m) {}
};

// Thrown when a blocking read is interrupted by a signal (Ctrl+C).
class Interrupted : public std::runtime_error {
public:
    Interrupted() : std::runtime_error("interrupted") {}
};

// Thrown when a read times out (see set_read_timeout).
class Timeout : public std::runtime_error {
public:
    explicit Timeout(const std::string& m) : std::runtime_error(m) {}
};

class Transport {
public:
    virtual ~Transport() = default;
    virtual void open() = 0;
    virtual void close() = 0;
    virtual void write(const Bytes& data) = 0;
    virtual Bytes read_exact(size_t n) = 0;
    // seconds <= 0 restores fully blocking reads (the default).
    virtual void set_read_timeout(double seconds) = 0;
};

class TcpTransport : public Transport {
public:
    TcpTransport(std::string host, int port, double connect_timeout = 5.0)
        : host_(std::move(host)), port_(port), connect_timeout_(connect_timeout) {}
    ~TcpTransport() override { close(); }

    void open() override;
    void close() override;
    void write(const Bytes& data) override;
    Bytes read_exact(size_t n) override;
    void set_read_timeout(double seconds) override;

private:
    std::string host_;
    int port_;
    double connect_timeout_;
    int fd_ = -1;
};

class SerialTransport : public Transport {
public:
    SerialTransport(std::string port, int baudrate = 115200)
        : port_(std::move(port)), baudrate_(baudrate) {}
    ~SerialTransport() override { close(); }

    void open() override;
    void close() override;
    void write(const Bytes& data) override;
    Bytes read_exact(size_t n) override;
    void set_read_timeout(double seconds) override;

private:
    std::string port_;
    int baudrate_;
    int fd_ = -1;
    bool read_timeout_set_ = false;
};

// kind == "tcp": "host:port" -> (host, port)
// kind == "serial": "/dev/ttyUSB0" or "COM6:115200" -> (port, baud)
std::pair<std::string, int> parse_endpoint(const std::string& spec, const std::string& kind);

}  // namespace rfid
