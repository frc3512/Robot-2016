// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "State.hpp"

/* The first state added to the state machine will be made the initial state.
 * Call Run() periodically and Transition() with the names of events that have
 * occurred to operate the state machine.
 *
 * Note: StateMachine::Run() should not be set by the user. Doing so will break
 * the state machine.
 */
class StateMachine : public State {
public:
    explicit StateMachine(std::string name);

    /* Ownership of 'state' will be transferred to this class, which will handle
     * destroying it. The first state added to the state machine will be made
     * the initial state.
     */
    void AddState(std::unique_ptr<State> state);

    template <class T, class = std::enable_if_t<
                           !std::is_same<std::decay_t<T>, State>::value>>
    void EmplaceState(T&& state) noexcept(
        std::is_nothrow_assignable<State&, T>::value) {
        m_states.push_back(std::make_unique<State>(std::forward<T>(state)));
    }

    // Returns name of current state
    std::string StackTrace() const override;

    /* If set to 'true', enables print statements for current state, event
     * occurrences, and transitons.
     */
    void EnableDebug(bool enable);

private:
    /* Moves the state machine to the given state. If the next state is found,
     * Exit() for the current state and Entry() for the next state are called.
     * 'true' is returned if the next state was found and 'false' otherwise.
     */
    bool SetState(const std::string& newState);

    std::vector<std::unique_ptr<State>> m_states;
    State* m_currentState = nullptr;
    bool m_debugEnabled = false;
    const std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::now();
};
