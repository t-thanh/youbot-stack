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

#include <youbot/DataTrace.hpp>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>

#include <std_msgs/Float64MultiArray.h>

#include <rtt/types/GlobalsRepository.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;

	class DataTracer: public TaskContext
	{
		public:
			DataTracer(const string& name);
			virtual ~DataTracer();

			enum setp_types {ANGULAR = 0, VELOCITY = 1, CURRENT=3, TORQUE=4, PWM=5};
			enum my_part {ARM = 0, BASE = 1};

		protected:
			unsigned int getJoint();
			virtual bool setJoint(my_part part, unsigned int joint_nr);

			virtual bool setpointType(unsigned int);

			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();

		private:
			youbot::DataTrace* m_trace;
			unsigned int m_joint_nr;
			my_part m_part;
			unsigned int m_setpoint_type;

			InputPort<std_msgs::Float64MultiArray> setpoint;
			std_msgs::Float64MultiArray m_setpoint;
	};
}
