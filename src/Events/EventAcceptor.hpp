// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

/**
 * Receives events
 */
class EventAcceptor {
public:
    virtual ~EventAcceptor() = default;

    virtual std::string HandleEvent(std::string& event) = 0;
};
