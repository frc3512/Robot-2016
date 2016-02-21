// =============================================================================
// Description: Base class for all robot subsystems
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SUBSYSTEM_BASE_HPP
#define SUBSYSTEM_BASE_HPP


class SubsystemBase {
public:
    virtual ~SubsystemBase() = default;

    virtual void ReloadPID() = 0;
    virtual void ResetEncoders() = 0;
};

#endif // SUBSYSTEM_BASE_HPP
