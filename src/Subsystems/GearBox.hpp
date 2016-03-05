// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef GEARBOX_HPP
#define GEARBOX_HPP

#include <DigitalInput.h>
#include <PIDOutput.h>
#include <PIDSource.h>
#include <Solenoid.h>

#include "../WPILib/CANTalon.hpp"

/* Notes:
 * This class uses only CANTalons.
 *
 * Up to three motors can be specified per gearbox, since drive train gearboxes
 * will use up to three and other gearboxes will use less.
 */

class GearBox : public PIDOutput, public PIDSource {
public:
    GearBox(int shifterChan,
            int forwardLimitPin,
            int reverseLimitPin,
            int motor1,
            int motor2 = -1,
            int motor3 = -1);

    // Disables PID controller and sets the motor speeds manually
    void Set(float value);

    // Returns current count on encoder
    int32_t Get() const;

    // Returns current position of master CANTalon
    float GetPosition() const;

    // Returns current speed of master CANTalon
    float GetSpeed() const;

    void SetDistancePerPulse(double distancePerPulse);

    // Resets encoder distance to 0
    void ResetEncoder();

    // Reverses gearbox drive direction
    void SetInverted(bool reverse);

    // Returns motor reversal state of gearbox
    bool GetInverted() const;

    // Reverses gearbox drive direction
    void SetSensorDirection(bool reverse);

    // Returns motor reversal state of gearbox
    bool IsEncoderReversed() const;

    // Shifts gearbox to another gear if available
    void SetGear(bool gear);

    // Gets current gearbox gear if available (false if not)
    bool GetGear() const;

    // Returns non-owning pointer to master CANTalon
    CANTalon* GetMaster() const;

    // PIDOutput interface
    void PIDWrite(float output) override;

    // PIDSource interface
    double PIDGet() override;

private:
    bool m_isEncoderReversed = false;

    // Conversion factor for setpoints with respect to encoder readings
    double m_distancePerPulse = 1.0;

    std::unique_ptr<Solenoid> m_shifter;

    // Prevents motor from rotating forward when switch is pressed
    std::unique_ptr<DigitalInput> m_forwardLimit;

    // Prevents motor from rotating in reverse when switch is pressed
    std::unique_ptr<DigitalInput> m_reverseLimit;

    std::vector<std::unique_ptr<CANTalon>> m_motors;
};

#endif // GEARBOX_HPP
