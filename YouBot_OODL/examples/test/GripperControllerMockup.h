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

#include "YouBotTypes.hpp"
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class GripperControllerMockup: public TaskContext
	{
		public:
		GripperControllerMockup(const string& name);
		virtual ~GripperControllerMockup();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

			void openGripper();
			void closeGripper();

		private:
//			InputPort< sensor_msgs::JointState > joint_states;

			OutputPort< motion_control_msgs::JointPositions > gripper_cmd_position;

//			sensor_msgs::JointState m_joint_states;

			motion_control_msgs::JointPositions m_gripper_cmd_position;

	};
}
