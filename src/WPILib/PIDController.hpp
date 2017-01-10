/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
#include <memory>

#include "Base.h"
#include "HAL/cpp/priority_mutex.h"
#include "LiveWindow/LiveWindow.h"
#include "Notifier.h"
#include "PIDInterface.hpp"
#include "PIDSource.h"
#include "Timer.h"

namespace frc {

class PIDOutput;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given
 * PIDOutput
 */
class PIDController : public LiveWindowSendable,
                      public PIDInterface,
                      public ITableListener {
public:
    PIDController(float p, float i, float d, PIDSource* source,
                  PIDOutput* output, float period = 0.05);
    PIDController(float p, float i, float d, float v, float a,
                  PIDSource* source, PIDOutput* output, float period = 0.05);
    virtual ~PIDController();

    PIDController(const PIDController&) = delete;
    PIDController& operator=(const PIDController) = delete;

    virtual float Get() const;
    virtual void SetContinuous(bool continuous = true);
    virtual void SetInputRange(float minimumInput, float maximumInput);
    virtual void SetOutputRange(float minimumOutput, float maximumOutput);
    virtual void SetPID(double p, double i, double d) override;
    virtual void SetPID(double p, double i, double d, double v, double a);
    virtual double GetP() const override;
    virtual double GetI() const override;
    virtual double GetD() const override;
    virtual double GetV() const;
    virtual double GetA() const;

    virtual void SetSetpoint(PIDState setpoint) override;
    virtual PIDState GetSetpoint() const override;

    virtual float GetError() const;

    virtual void SetPIDSourceType(PIDSourceType pidSource);
    virtual PIDSourceType GetPIDSourceType() const;

    virtual void SetTolerance(float percent);
    virtual void SetAbsoluteTolerance(float absValue);
    virtual void SetPercentTolerance(float percentValue);
    virtual bool OnTarget() const;

    virtual void Enable() override;
    virtual void Disable() override;
    virtual bool IsEnabled() const override;

    virtual void Reset() override;

    virtual void InitTable(std::shared_ptr<ITable> table) override;

protected:
    PIDSource* m_pidInput;
    PIDOutput* m_pidOutput;

    std::shared_ptr<ITable> m_table;
    virtual void Calculate();
    virtual double CalculateFeedForward();

private:
    double m_P;  // factor for "proportional" control
    double m_I;  // factor for "integral" control
    double m_D;  // factor for "derivative" control
    double m_V;  // factor for "velocity feed forward" control
    double m_A;  // factor for "acceleration feed forward" control
    float m_maximumOutput = 1.0;   // |maximum output|
    float m_minimumOutput = -1.0;  // |minimum output|
    float m_maximumInput = 0;      // maximum input - limit setpoint to this
    float m_minimumInput = 0;      // minimum input - limit setpoint to this
    bool m_continuous =
        false;  // do the endpoints wrap around? eg. Absolute encoder
    bool m_enabled = false;  // is the pid controller enabled
    float m_prevError = 0;   // the prior error (used to compute velocity)
    double m_totalError =
        0;  // the sum of the errors for use in the integral calc
    enum {
        kAbsoluteTolerance,
        kPercentTolerance,
        kNoTolerance
    } m_toleranceType = kNoTolerance;

    // the percentage or absolute error that is considered on target.
    float m_tolerance = 0.05;
    PIDState m_setpoint;
    PIDState m_prevSetpoint;
    float m_error = 0;
    float m_result = 0;
    float m_period;

    mutable priority_recursive_mutex m_mutex;

    std::unique_ptr<Notifier> m_controlLoop;

    void Initialize(float p, float i, float d, float v, float a,
                    PIDSource* source, PIDOutput* output, float period = 0.05);

    virtual std::shared_ptr<ITable> GetTable() const override;
    virtual std::string GetSmartDashboardType() const override;
    virtual void ValueChanged(ITable* source, llvm::StringRef key,
                              std::shared_ptr<nt::Value> value,
                              bool isNew) override;
    virtual void UpdateTable() override;
    virtual void StartLiveWindowMode() override;
    virtual void StopLiveWindowMode() override;
};

}  // namespace frc
