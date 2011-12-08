#pragma once

#include <rtt/RTT.hpp>

#include <YouBotTypes.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class TSimToYouBotMsg : public RTT::TaskContext
	{
		public:
			TSimToYouBotMsg(string const& name);
			~TSimToYouBotMsg();

			void initialize(ctrl_modes ctrl_mode, unsigned int dimension = 6);

			bool startHook() ;
			void updateHook() ;

		private:
			InputPort<std_msgs::Float64MultiArray > input_cmd_signal;

			OutputPort<motion_control_msgs::JointPositions> output_cmd_angles;
			OutputPort<motion_control_msgs::JointVelocities> output_cmd_velocities;
			OutputPort<motion_control_msgs::JointEfforts> output_cmd_torques;
			OutputPort<geometry_msgs::Twist> output_cmd_twist;

			std_msgs::Float64MultiArray m_input_cmd_signal;

			motion_control_msgs::JointVelocities m_output_cmd_velocities;
			motion_control_msgs::JointPositions m_output_cmd_angles;
			motion_control_msgs::JointEfforts m_output_cmd_torques;
			geometry_msgs::Twist m_output_cmd_twist;

			ctrl_modes m_ctrl_mode;

			unsigned int m_dimension;
	};
}

