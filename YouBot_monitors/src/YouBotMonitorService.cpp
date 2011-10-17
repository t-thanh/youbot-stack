#include "YouBotMonitorService.hpp"

#include <stdio.h>
#include <cassert>

#include <boost/lexical_cast.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	/**
	 * @brief Generic services for YouBot.
	 */
	YouBotMonitorService::YouBotMonitorService(const string& name, TaskContext* parent) :
			Service(name, parent)
	{
		memset(&m_events.stamp, 0, sizeof(ros::Time));
		m_events.monitor_event.reserve(max_event_length);

        // Pre-allocate port memory for outputs
		events.setDataSample(m_events);
	}

	YouBotMonitorService::~YouBotMonitorService()
	{ }

	void YouBotMonitorService::setupComponentInterface()
	{
		this->addPort("events", events).doc("Joint events");
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message)
	{
		m_events.stamp = ros::Time::now();
		m_events.monitor_event = id + "." + message; //"jnt" + boost::lexical_cast<string>(joint)
		events.write(m_events);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message, bool condition)
	{
		m_events.stamp = ros::Time::now();
		m_events.monitor_event = id + "." + message + "_" + (condition ? "true" : "false");
		events.write(m_events);
	}

}
