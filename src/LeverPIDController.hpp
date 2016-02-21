// =============================================================================
// Description: Gravity compensation feed forward for lever arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "WPILib/PIDController.hpp"

class LeverPIDController : public PIDController {
public:
    LeverPIDController(float p, float i, float d, float v, float a, float f,
                       PIDSource* source, PIDOutput* output,
                       float period = 0.05);
    void SetF(double f);
    double GetF() const;
    double CalculateFeedForward();

private:
    double m_F;
};
