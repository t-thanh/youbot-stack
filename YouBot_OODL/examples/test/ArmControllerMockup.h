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

	class ArmControllerMockup: public TaskContext
	{
		public:
		ArmControllerMockup(const string& name);
		virtual ~ArmControllerMockup();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

			void unfoldManipulator();
			void setJointAngles(vector<double>& angles, double epsilon);
			bool getJointAngles(vector<double>& angles);

		private:
			InputPort< sensor_msgs::JointState > joint_states;

			InputPort< YouBot_OODL::motor_statuses > joint_statuses;

			OutputPort< motion_control_msgs::JointPositions > joint_cmd_angles;

			OutputPort< vector<ctrl_modes> > joint_ctrl_modes;

			sensor_msgs::JointState m_joint_states;

			YouBot_OODL::motor_statuses m_joint_statuses;
			vector<ctrl_modes> m_modes;

			motion_control_msgs::JointPositions m_joint_cmd_angles;
//			vector<quantity<si::angular_velocity> > m_joint_cmd_velocities;
//			vector<quantity<si::torque> > m_joint_cmd_torques;

	};
}
