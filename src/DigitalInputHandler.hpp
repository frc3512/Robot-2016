// =============================================================================
// Description: Provides an interface for sharing digital inputs between
//              subsystems and the event framework
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef DIGITAL_INPUT_HANDLER
#define DIGITAL_INPUT_HANDLER

#include <memory>
#include <vector>

#include <DigitalInput.h>

class DigitalInputHandler {
public:
    static DigitalInput* Get(uint32_t channel);

private:
    static std::vector<std::unique_ptr<DigitalInput>> m_inputs;
};

#endif // DIGITAL_INPUT_HANDLER
