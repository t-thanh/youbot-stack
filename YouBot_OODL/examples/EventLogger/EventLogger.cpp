#include "EventLogger.h"

#include <ocl/Component.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	EventLogger::EventLogger(const string& name) :
			TaskContext(name)
	{
		m_events.assign(255, ' '); //@TODO: Fix me

		this->addEventPort("events",events).doc("Connect an event-emitting port to it to print events on the screen.");
	}

	EventLogger::~EventLogger() {}

	bool EventLogger::startHook()
	{
		if(! events.connected())
		{
			log(Error) << "Event port not connected." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void EventLogger::updateHook()
	{
        TaskContext::updateHook();

        while(events.read(m_events) == NewData)
        {
        	log(Info) << m_events << endlog();
        }
	}
}

ORO_CREATE_COMPONENT( YouBot::EventLogger )
