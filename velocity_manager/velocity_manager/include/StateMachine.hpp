#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <iostream>
#include <tuple>
#include <variant>
#include <functional>

struct AcceptAllState; //default state
struct DeadManState;
struct JoyState;
struct AutonomousState;
constexpr bool VERBOSE = true;
constexpr bool DEBUG = false;

#define SHELL_ABORT_GOAL "\
#/bin/bash \n\
rostopic pub --once /move_base/cancel actionlib_msgs/GoalID -- {} \n\
"

template <typename... States>
class StateMachine
{
public:
    template <typename State>
    void transitionTo()
    {
        current_state = &std::get<State>(states);
    }
    template <typename Event>
    void handle(const Event &event)
    {
        auto passEventToState = [this, &event](auto statePtr)
        {
            statePtr->handle(event).execute(*this);
        };
        std::visit(passEventToState, current_state);
    }
    int getCStateIndex()
    {
        const int c_state = current_state.index();
        return c_state;
    }

private:
    // std::tuple public : StateMachine(/* args */);
    std::tuple<States...> states;
    std::variant<States *...> current_state{&std::get<0>(states)};
};

template <typename State>
struct TransitionTo
{
    template <typename Machine>
    void execute(Machine &machine)
    {
        machine.template transitionTo<State>();
    }
};

template <typename Action>
struct ByDefault
{
    template <typename Event>
    Action handle(const Event &) const
    {
        return Action{};
    }
};

// transit to the same state as actual
struct Nothing
{
    template <typename Machine>
    void execute(Machine &)
    {
    }
};

struct TimeoutEvent
{
};
struct AcceptAllEvent
{
};
struct EnableDeadManEvent
{
};
struct AutonomousEvent
{
};
struct DeadManEvent
{
};
struct CmdEvent
{
};
struct JoyCmdEvent
{
};

struct AcceptAllState : ByDefault<Nothing>
{
    using ByDefault::handle;
    // change to other state by handler
    TransitionTo<DeadManState> handle(const EnableDeadManEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AcceptAllState (EnableDeadManEvent) ->  DeadManState" << std::endl;
        }
        return {};
    }

    TransitionTo<JoyState> handle(const JoyCmdEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AcceptAllState (JoyCmdEvent) ->  JoyState" << std::endl;
        }
        return {};
    }

    TransitionTo<AutonomousState> handle(const AutonomousEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AcceptAllState (AutonomousEvent) ->  AutonomousState" << std::endl;
        }
        return {};
    }
    // dont change state on nothing events
};

struct DeadManState : ByDefault<Nothing>
{
    using ByDefault::handle;
    TransitionTo<AcceptAllState> handle(const AcceptAllEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "DeadManState (AcceptAllEvent) ->  AcceptAllState" << std::endl;
        }
        return {};
    }
};

struct JoyState : ByDefault<Nothing>
{
    using ByDefault::handle;
    TransitionTo<AcceptAllState> handle(const AcceptAllEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "JoyState (AcceptAllEvent) ->  AcceptAllState" << std::endl;
        }
        return {};
    }

    TransitionTo<AcceptAllState> handle(const TimeoutEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "JoyState (TimeoutEvent) ->  AcceptAllState" << std::endl;
        }
        return {};
    }

    TransitionTo<DeadManState> handle(const EnableDeadManEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "JoyState (EnableDeadManEvent) ->  DeadManState" << std::endl;
        }
        return {};
    }
};

struct AutonomousState : ByDefault<Nothing>
{
    using ByDefault::handle;
    TransitionTo<AcceptAllState> handle(const TimeoutEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AutonomousState (TimeoutEvent) ->  AcceptAllState" << std::endl;
            std::cout << "aborting goal" << std::endl;
        }
        system(SHELL_ABORT_GOAL);
        return {};
    }

    TransitionTo<AcceptAllState> handle(const CmdEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AutonomousState (CmdEvent) ->  AcceptAllState" << std::endl;
        }
        system(SHELL_ABORT_GOAL);
        return {};
    }

    TransitionTo<DeadManState> handle(const DeadManEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AutonomousState (DeadManEvent) ->  DeadManState" << std::endl;
        }
        system(SHELL_ABORT_GOAL);
        return {};
    }

    TransitionTo<JoyState> handle(const JoyCmdEvent &) const
    {
        if (VERBOSE)
        {
            std::cout << "AutonomousState (JoyCmdEvent) ->  JoyState" << std::endl;
        }
        system(SHELL_ABORT_GOAL);
        return {};
    }
};

#endif