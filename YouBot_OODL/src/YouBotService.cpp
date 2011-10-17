#include "YouBotService.hpp"

#include <stdio.h>
#include <cassert>

#include <boost/lexical_cast.hpp>

#include "YouBotHelpers.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	/**
	 * @brief Generic services for YouBot.
	 */
	YouBotService::YouBotService(const string& name, TaskContext* parent) :
			Service(name, parent)
	{
		memset(&m_events.stamp, 0, sizeof(ros::Time));
		m_events.driver_event.reserve(max_event_length);

        // Pre-allocate port memory for outputs
		events.setDataSample(m_events);
	}

	YouBotService::~YouBotService()
	{ }

	void YouBotService::setupComponentInterface()
	{
		this->addPort("events", events).doc("Joint events");
	}

	void YouBotService::emitEvent(std::string id, std::string message)
	{
		m_events.stamp = ros::Time::now();
		m_events.driver_event = id + "." + message; //"jnt" + boost::lexical_cast<string>(joint)
		events.write(m_events);
	}

	void YouBotService::emitEvent(std::string id, std::string message, bool condition)
	{
		m_events.stamp = ros::Time::now();
		m_events.driver_event = id + "." + message + "_" + (condition ? "true" : "false");
		events.write(m_events);
	}

	// Joints from 0 to N-1
	void check_event_edge(YouBotService* const serv, const motor_status ref_cond, const std::string outp_message, bool* const cond_state,
			unsigned int joint, motor_status current)
	{
		if((ref_cond & current) != 0 && !(cond_state[joint]) )
		{
			cond_state[joint] = true;
			serv->emitEvent("jnt" + boost::lexical_cast<string>(joint+1), outp_message, true);
		}
		else if(cond_state[joint] && (ref_cond & current) == 0)
		{
			cond_state[joint] = false;
			serv->emitEvent("jnt" + boost::lexical_cast<string>(joint+1), outp_message, false);
		}
	}

	void check_event_level(YouBotService* const serv, const motor_status ref_cond, const std::string outp_message,
			unsigned int joint, motor_status current)
	{
		if((ref_cond & current) != 0)
		{
			serv->emitEvent("jnt" + boost::lexical_cast<string>(joint+1), outp_message);
		}
	}

}
