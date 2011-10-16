#pragma once

#include <YouBot_OODL/typekit/Types.h>

namespace YouBot
{
	enum ctrl_modes {PLANE_ANGLE = 1, ANGULAR_VELOCITY = 2 , TORQUE = 3, MOTOR_STOP = 4, TWIST = 5};

	typedef YouBot_OODL::motor_statuses::_flags_type::value_type motor_status;
}
