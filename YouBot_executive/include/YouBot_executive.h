#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
	using namespace RTT;
	typedef std_msgs::Float64MultiArray flat_matrix_t;
	class YouBot_executive: public TaskContext
	{
		public:
		YouBot_executive(const string& name);
		virtual ~YouBot_executive();
		void unfoldArm();
		RTT::OutputPort<flat_matrix_t > JointSpaceSetpoint;
		RTT::OutputPort<flat_matrix_t > JointSpaceStiffnes;
		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();


	};
}
