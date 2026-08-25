// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace server {

/// Move-only TCP client for the server's length-prefixed message protocol.
class TcpLpmClient {
public:
    TcpLpmClient(std::string host, int port);
    explicit TcpLpmClient(int adopted_socket_fd);
    ~TcpLpmClient();

    TcpLpmClient(const TcpLpmClient&) = delete;
    TcpLpmClient& operator=(const TcpLpmClient&) = delete;

    TcpLpmClient(TcpLpmClient&& other) noexcept;
    TcpLpmClient& operator=(TcpLpmClient&& other) noexcept;

    void connect();
    /// Sends one message with a 4-byte big-endian size prefix.
    void send_message(const std::string& message) const;
    /// Receives one complete length-prefixed message.
    std::string receive_message() const;
    void close();

    static std::array<std::uint8_t, 4> encode_message_size(std::uint32_t size);
    static std::uint32_t decode_message_size(const std::array<std::uint8_t, 4>& prefix);

private:
    std::string host_;
    int port_{0};
    int socket_fd_{-1};

    void ensure_connected() const;
};

}  // namespace server
