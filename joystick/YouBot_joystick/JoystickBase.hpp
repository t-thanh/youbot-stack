#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include <YouBotTypes.hpp>
#include <joy/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class JoystickBase: public TaskContext
	{
		public:
			JoystickBase(const string& name);
			virtual ~JoystickBase();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
			InputPort<joy::Joy> joystick;
			InputPort< nav_msgs::Odometry > odometry_state;

			OutputPort< geometry_msgs::Twist > cmd_twist;

			nav_msgs::Odometry m_odometry_state;

			geometry_msgs::Twist m_cmd_twist;
			joy::Joy m_joystick;

			OperationCaller<void(vector<ctrl_modes>) > op_setControlModes;

			vector<ctrl_modes> m_modes;

			double m_K[3];
			double m_error[3];
	};
}
