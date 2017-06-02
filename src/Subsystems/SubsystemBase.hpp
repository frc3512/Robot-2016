// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#pragma once

/**
 * Base class for all robot subsystems
 */
class SubsystemBase {
public:
    virtual ~SubsystemBase() = default;

    virtual void ReloadPID() = 0;
    virtual void ResetEncoders() = 0;
};
