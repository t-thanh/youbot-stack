#pragma once

#include <rtt/RTT.hpp>

#include <YouBotTypes.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

namespace YouBot
{
	using namespace RTT;

	class YouBotOdometryToTSim : public RTT::TaskContext
	{
		public:
			YouBotOdometryToTSim(string const& name);
			~YouBotOdometryToTSim();

			void initialize();

			bool startHook() ;
			void updateHook() ;

		private:
			OutputPort<std_msgs::Float64MultiArray > output_odometry;

			InputPort< nav_msgs::Odometry > input_odometry;

			std_msgs::Float64MultiArray m_output_odometry;

			nav_msgs::Odometry m_input_odometry;

			unsigned int m_dimension;
	};
}

