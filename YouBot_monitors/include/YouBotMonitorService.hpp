#pragma once

#include <YouBot_monitors/typekit/Types.h>
#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	const size_t max_event_length = 255;

    class YouBotMonitorService : public Service {

		public:
    		YouBotMonitorService(const string& name, TaskContext* parent);
			virtual ~YouBotMonitorService();

			virtual void setupComponentInterface();
			void emitEvent(std::string id, std::string message);
			void emitEvent(std::string id, std::string message, bool condition);

		protected:

			OutputPort<YouBot_monitors::monitor_event> events;
			YouBot_monitors::monitor_event m_events;
    };

}
