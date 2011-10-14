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
			InputPort<YouBot_OODL::driver_event> events;
			YouBot_OODL::driver_event m_events;

			ros::Time begin;
	};
}
