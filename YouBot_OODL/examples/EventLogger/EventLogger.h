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

#include <std_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class EventLogger: public TaskContext
	{
		public:
		EventLogger(const string& name);
		virtual ~EventLogger();

		protected:
			virtual bool startHook();
			virtual void updateHook();

		private:
			InputPort<std::string> events;
			std::string m_events;
	};
}
