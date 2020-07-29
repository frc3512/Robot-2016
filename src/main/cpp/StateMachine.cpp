// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "StateMachine.hpp"

#include <iostream>
#include <string>
#include <utility>

StateMachine::StateMachine(std::string name) : State(std::move(name)) {
    Run = [this] { m_currentState->Run(); };
}

void StateMachine::AddState(std::unique_ptr<State> state) {
    // First state added is made the initial state
    if (m_states.empty()) {
        m_currentState = state.get();
    }

    m_states.push_back(std::move(state));
}

std::string StateMachine::StackTrace() const {
    // Prepend state machine's name to provide a stack trace
    return Name() + " > " + m_currentState->StackTrace();
}

bool StateMachine::SetState(const std::string& newState) {
    for (auto& state : m_states) {
        if (state->Name() == newState) {
            if (m_currentState != nullptr) {
                m_currentState->Exit();
            }
            m_currentState = state.get();
            m_currentState->Entry();

            return true;
        }
    }

    return false;
}

void StateMachine::EnableDebug(bool enable) { m_debugEnabled = enable; }
