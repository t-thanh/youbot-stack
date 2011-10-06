#pragma once

#include <rtt/RTT.hpp>

#include <YouBotTypes.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

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
			OutputPort<std::vector<double> > output_positions;
			OutputPort<std::vector<double> > output_velocities;
			OutputPort<std::vector<double> > output_torques;

			InputPort< sensor_msgs::JointState > input_states;

			std::vector<double> m_output_positions;
			std::vector<double> m_output_velocities;
			std::vector<double> m_output_torques;

			sensor_msgs::JointState m_input_states;

			unsigned int m_dimension;
	};
}

