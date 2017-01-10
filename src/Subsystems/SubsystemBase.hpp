// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

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
