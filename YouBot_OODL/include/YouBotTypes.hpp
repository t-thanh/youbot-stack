#pragma once

#include <YouBot_OODL/typekit/Types.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace YouBot
{
	enum arm_settings {NR_OF_ARM_SLAVES=5};
	enum base_settings {NR_OF_BASE_SLAVES=4};

	enum ctrl_modes {PLANE_ANGLE = 1, ANGULAR_VELOCITY = 2 , TORQUE = 3, MOTOR_STOP = 4, TWIST = 5};

	typedef YouBot_OODL::motor_statuses::_flags_type::value_type motor_status;

	typedef boost::function<void(unsigned int motor_id, motor_status current)> check_fp;
}
