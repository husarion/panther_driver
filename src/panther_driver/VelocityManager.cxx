#include "VelocityManager.hpp"

VelocityManager::VelocityManager(/* args */)
{
    vm_sm;
    this->last_msg_time = std::chrono::system_clock::now();
}

VelocityManager::~VelocityManager()
{
}

CmdVelInfo VelocityManager::getVelocity()
{
    int state = vm_sm.getCStateIndex();
    CmdVelInfo resp{};
    resp = currentVel;
    return resp;
}

std::string VelocityManager::getCurrentDescription()
{
    std::string state = vm_sm.getCStateName();
    return state;
}

uint VelocityManager::getCurrentIndex()
{
    uint state = vm_sm.getCStateIndex();
    // std::cout << "State index: " << state << std::endl;
    return state;
}

void VelocityManager::updateJoy(std::vector<int> j_buttons)
{
    this->last_msg_time = std::chrono::system_clock::now();
    if (j_buttons.at(nr_enable_dm))
    {
        vm_sm.handle(EnableDeadManEvent{});
    }
    if (j_buttons.at(nr_disable_dm))
    {
        vm_sm.handle(AcceptAllEvent{});
    }
    if (j_buttons.at(dm_hold_button))
    {
        vm_sm.handle(DeadManEvent{});
    }
    dm_pressed = j_buttons.at(dm_hold_button); // set state of dead man
}

void VelocityManager::updateCmdVel(CmdVelInfo cvi)
{
    this->last_msg_time = std::chrono::system_clock::now();
    if (cvi.publisher_name == "f710_teleop_joy_node")
    {
        vm_sm.handle(JoyCmdEvent{});
        const int state = vm_sm.getCStateIndex();
        // std::cout << "State index: " << state << std::endl;
        if (state == 2)
        {
            currentVel = cvi;
        }
    }
    else if (cvi.publisher_name == "autonomous")
    {
        vm_sm.handle(AutonomousEvent{});
        const int state = vm_sm.getCStateIndex();
        if (state == 3)
        {
            currentVel = cvi;
        }
        else if (state == 1 && dm_pressed)
        {
            currentVel = cvi;
        }
        else
        {
            currentVel.reset();
        }
    }
    else
    {
        vm_sm.handle(CmdEvent{});
        const int state = vm_sm.getCStateIndex();
        if (state == 0)
        {
            currentVel = cvi;
        }
    }
}

void VelocityManager::spin()
{
    auto time_now = std::chrono::system_clock::now();

    if (time_now - this->last_msg_time >= ((std::chrono::seconds{1}) / reset_vel_hz))
    {
        //no timeout --> no change in status
        currentVel.reset(); //reset vel when no new messages
    }
    if (time_now - this->last_msg_time >= std::chrono::seconds{timeout_s})
    {
        this->last_msg_time = time_now;
        vm_sm.handle(TimeoutEvent{});
        currentVel.reset(); //reset vel -> prevent const vel to be published
    }
}