#pragma once

#include <rtt/RTT.hpp>

#include <YouBotTypes.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
	using namespace RTT;

	class YouBotMsgToTSim : public RTT::TaskContext
	{
		public:
			YouBotMsgToTSim(string const& name);
			~YouBotMsgToTSim();

			void initialize(unsigned int dimension = 6);

			bool startHook() ;
			void updateHook() ;

		private:
			OutputPort<std_msgs::Float64MultiArray > output_positions;
			OutputPort<std_msgs::Float64MultiArray > output_velocities;
			OutputPort<std_msgs::Float64MultiArray > output_torques;

			InputPort< sensor_msgs::JointState > input_states;

			std_msgs::Float64MultiArray m_output_positions;
			std_msgs::Float64MultiArray m_output_velocities;
			std_msgs::Float64MultiArray m_output_torques;

			sensor_msgs::JointState m_input_states;

			unsigned int m_dimension;
	};
}

