#pragma once

//#include "orocos/YouBot_messages/Types.hpp"

//using namespace RTT::types;

namespace YouBot
{

	typedef unsigned int joint_status; //filtered version of the OODL statuses -> no POSITION_REACHED etc.

	enum ctrl_modes {PLANE_ANGLE = 1, ANGULAR_VELOCITY = 2 , TORQUE = 3, MOTOR_STOP = 4};
}
