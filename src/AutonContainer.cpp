// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "AutonContainer.hpp"

AutonMethod::AutonMethod(const std::string& methodName,
                         std::function<void()> func) {
    name = methodName;
    function = func;
}

void AutonContainer::AddMethod(const std::string& methodName,
                               std::function<void()> func) {
    m_functionList.emplace_back(methodName, func);
}

void AutonContainer::DeleteAllMethods() { m_functionList.clear(); }

size_t AutonContainer::Size() const { return m_functionList.size(); }

const std::string& AutonContainer::Name(size_t pos) {
    return m_functionList[pos].name;
}

void AutonContainer::ExecAutonomous(size_t pos) {
    // Retrieves correct autonomous routine
    auto& auton = m_functionList[pos];

    // Runs the routine
    (auton.function)();
}
