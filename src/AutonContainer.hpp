// =============================================================================
// Description: Stores Autonomous modes as function pointers for easy retrieval
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef AUTON_CONTAINER_HPP
#define AUTON_CONTAINER_HPP

#include <vector>
#include <string>
#include <functional>

struct AutonMethod {
    std::string name;
    std::function<void()> function;

    AutonMethod(const std::string& methodName, std::function<void()> func);
};

class AutonContainer {
public:
    // Add and remove autonomous functions
    void AddMethod(const std::string& methodName,
                   std::function<void()> func);
    void DeleteAllMethods();

    // Returns number of routines currently held
    size_t Size() const;

    // Returns name of specific autonomous function
    const std::string& Name(size_t pos);

    // Run specific autonomous function
    void ExecAutonomous(size_t pos);

private:
    // Contains function pointers to the autonomous functions
    std::vector<AutonMethod> m_functionList;
};

#endif // AUTON_CONTAINER_HPP
