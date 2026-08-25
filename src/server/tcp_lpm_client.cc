// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/server/tcp_lpm_client.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

namespace server {

namespace {

void send_all(int socket_fd, const void* buffer, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(buffer);
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t result = ::write(socket_fd, bytes + sent, size - sent);
        if (result <= 0) {
            throw std::runtime_error(std::string{"write failed: "} + std::strerror(errno));
        }
        sent += static_cast<std::size_t>(result);
    }
}

void recv_all(int socket_fd, void* buffer, std::size_t size) {
    auto* bytes = static_cast<std::uint8_t*>(buffer);
    std::size_t received = 0;
    while (received < size) {
        const ssize_t result = ::read(socket_fd, bytes + received, size - received);
        if (result <= 0) {
            throw std::runtime_error(std::string{"read failed: "} + std::strerror(errno));
        }
        received += static_cast<std::size_t>(result);
    }
}

}  // namespace

TcpLpmClient::TcpLpmClient(std::string host, int port)
    : host_(std::move(host)),
      port_(port) {}

TcpLpmClient::TcpLpmClient(int adopted_socket_fd)
    : socket_fd_(adopted_socket_fd) {}

TcpLpmClient::~TcpLpmClient() {
    close();
}

TcpLpmClient::TcpLpmClient(TcpLpmClient&& other) noexcept
    : host_(std::move(other.host_)),
      port_(other.port_),
      socket_fd_(other.socket_fd_) {
    other.socket_fd_ = -1;
}

TcpLpmClient& TcpLpmClient::operator=(TcpLpmClient&& other) noexcept {
    if (this != &other) {
        close();
        host_ = std::move(other.host_);
        port_ = other.port_;
        socket_fd_ = other.socket_fd_;
        other.socket_fd_ = -1;
    }
    return *this;
}

void TcpLpmClient::connect() {
    if (socket_fd_ >= 0) {
        return;
    }
    if (port_ <= 0 || port_ > 65535) {
        throw std::invalid_argument("port must be in 1..65535");
    }

    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error(std::string{"socket failed: "} + std::strerror(errno));
    }

    int enabled = 1;
    if (::setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0) {
        close();
        throw std::runtime_error(std::string{"setsockopt(TCP_NODELAY) failed: "} + std::strerror(errno));
    }

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    const std::string service = std::to_string(port_);
    const int gai_error = ::getaddrinfo(host_.c_str(), service.c_str(), &hints, &result);
    if (gai_error != 0) {
        close();
        throw std::runtime_error(std::string{"getaddrinfo failed: "} + ::gai_strerror(gai_error));
    }

    bool connected = false;
    for (addrinfo* candidate = result; candidate != nullptr; candidate = candidate->ai_next) {
        if (::connect(socket_fd_, candidate->ai_addr, candidate->ai_addrlen) == 0) {
            connected = true;
            break;
        }
    }
    ::freeaddrinfo(result);

    if (!connected) {
        const std::string error_message = std::strerror(errno);
        close();
        throw std::runtime_error(std::string{"connect failed: "} + error_message);
    }
}

void TcpLpmClient::send_message(const std::string& message) const {
    ensure_connected();

    const auto prefix = encode_message_size(static_cast<std::uint32_t>(message.size()));
    send_all(socket_fd_, prefix.data(), prefix.size());
    if (!message.empty()) {
        send_all(socket_fd_, message.data(), message.size());
    }
}

std::string TcpLpmClient::receive_message() const {
    ensure_connected();

    std::array<std::uint8_t, 4> prefix{};
    recv_all(socket_fd_, prefix.data(), prefix.size());
    const std::uint32_t size = decode_message_size(prefix);

    std::string message(size, '\0');
    if (size > 0) {
        recv_all(socket_fd_, message.data(), size);
    }
    return message;
}

void TcpLpmClient::close() {
    if (socket_fd_ >= 0) {
        ::shutdown(socket_fd_, SHUT_RDWR);
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

void TcpLpmClient::ensure_connected() const {
    if (socket_fd_ < 0) {
        throw std::runtime_error("TcpLpmClient is not connected");
    }
}

std::array<std::uint8_t, 4> TcpLpmClient::encode_message_size(std::uint32_t size) {
    return {
        static_cast<std::uint8_t>((size >> 24U) & 0xFFU),
        static_cast<std::uint8_t>((size >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((size >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(size & 0xFFU),
    };
}

std::uint32_t TcpLpmClient::decode_message_size(const std::array<std::uint8_t, 4>& prefix) {
    return (static_cast<std::uint32_t>(prefix[0]) << 24U) |
           (static_cast<std::uint32_t>(prefix[1]) << 16U) |
           (static_cast<std::uint32_t>(prefix[2]) << 8U) |
           static_cast<std::uint32_t>(prefix[3]);
}

}  // namespace server
