// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef AUTON_CONTAINER_HPP
#define AUTON_CONTAINER_HPP

#include <functional>
#include <string>
#include <vector>

struct AutonMethod {
    std::string name;
    std::function<void()> function;

    AutonMethod(const std::string& methodName, std::function<void()> func);
};

/**
 * Stores Autonomous modes as function pointers for easy retrieval
 */
class AutonContainer {
public:
    // Add and remove autonomous functions
    void AddMethod(const std::string& methodName, std::function<void()> func);
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

#endif  // AUTON_CONTAINER_HPP
