// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <string>

template <class T>
void DSDisplay::AddAutoMethod(const std::string& methodName,
                              void (T::*function)(), T* object) {
    m_autonModes.AddMethod(methodName, std::bind(function, object));
}
