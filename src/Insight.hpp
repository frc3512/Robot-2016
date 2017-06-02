// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "SFML/Network/IpAddress.hpp"
#include "SFML/Network/UdpSocket.hpp"

/**
 * Receives Insight's processed target data
 */
class Insight {
public:
    Insight(const Insight&) = delete;
    Insight& operator=(const Insight&) = delete;
    virtual ~Insight();

    static Insight& GetInstance(uint16_t dsPort);

    // Receives control commands from Driver Station and processes them
    std::string ReceiveFromDS();

    // Returns true if new target data has been received
    bool HasNewData() const;

    // Provides access to target data
    const std::pair<char, char>& GetTarget(size_t i);
    size_t GetNumTargets() const;

private:
    explicit Insight(uint16_t portNumber);

    sf::UdpSocket m_socket;
    sf::IpAddress m_recvIP;  // stores IP address temporarily during receive
    uint16_t m_recvPort;     // stores port temporarily during receive

    char m_recvBuffer[256];  // buffer for Insight packets
    size_t m_recvAmount;  // holds number of bytes received from Driver Station

    std::vector<std::pair<char, char>> m_targets;
    bool m_hasNewData;

    static constexpr int k_numTargets = 1;
};
