/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Base.h"
#include "Controller.h"
#include "LiveWindow/LiveWindow.h"
#include "PIDState.hpp"

class PIDInterface : public Controller {
public:
    virtual void SetPID(double p, double i, double d) = 0;
    virtual void SetPID(double p, double i, double d, double v, double a) = 0;
    virtual double GetP() const = 0;
    virtual double GetI() const = 0;
    virtual double GetD() const = 0;
    virtual double GetV() const = 0;
    virtual double GetA() const = 0;

    virtual void SetSetpoint(PIDState setpoint) = 0;
    virtual PIDState GetSetpoint() const = 0;

    virtual void Enable() = 0;
    virtual void Disable() = 0;
    virtual bool IsEnabled() const = 0;

    virtual void Reset() = 0;
};
