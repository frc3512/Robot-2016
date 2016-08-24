// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef SUBSYSTEM_BASE_HPP
#define SUBSYSTEM_BASE_HPP

/**
 * Base class for all robot subsystems
 */
class SubsystemBase {
public:
    virtual ~SubsystemBase() = default;

    virtual void ReloadPID() = 0;
    virtual void ResetEncoders() = 0;
};

#endif  // SUBSYSTEM_BASE_HPP
