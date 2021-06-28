#include <iostream>
#include <tuple>
#include <variant>
#include <functional>
#include <chrono>
#include "StateMachine.hpp"

#ifndef VELOCITY_MANAGER_HPP
#define VELOCITY_MANAGER_HPP

struct Linear
{
    float x;
    float y;
    float z;
};

struct Angular
{
    float x;
    float y;
    float z;
};

enum class states
{
    AcceptAll,
    Joy,
    DeadMan,
    Autonomous
};

struct CmdVelInfo
{
    std::string publisher_name;
    Linear linear;
    Angular angular;
    void reset(){
        linear.x = 0;
        linear.y = 0;
        linear.z = 0;
        angular.x = 0;
        angular.y = 0;
        angular.z = 0;
        publisher_name = "reset";
    }
};

class VelocityManager
{
public:
    VelocityManager();
    ~VelocityManager();
    std::string getCurrentDescription();
    uint getCurrentIndex();
    void updateCmdVel(CmdVelInfo);
    void updateJoy(std::vector<int>);
    void spin();
    CmdVelInfo getVelocity();

private:
    void getNewData();
    StateMachine<AcceptAllState, DeadManState, JoyState, AutonomousState> vm_sm;
    std::chrono::time_point<std::chrono::system_clock> last_msg_time;
    int timeout_s = {5};
    float reset_vel_hz = {5}; //when no new data at this frequency then reset velocity

    // JOY buttons
    int nr_enable_dm = {0};
    int nr_disable_dm = {3};
    int dm_hold_button = {1};
    int reset_button = {2};

    volatile bool dm_pressed ={0}; // flag for detecting if dead man is pressed 
    CmdVelInfo currentVel{};
};
#endif