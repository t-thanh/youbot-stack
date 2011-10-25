#pragma once

#include <std_msgs/typekit/Types.h>

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;

	const size_t max_event_length = 255;

    class YouBotService : public Service {

		public:
    		YouBotService(const string& name, TaskContext* parent);
			virtual ~YouBotService();

			virtual void setupComponentInterface();
			void emitEvent(std::string id, std::string message);
			void emitEvent(std::string id, std::string message, bool condition);

		protected:
			virtual bool calibrate() = 0;
			virtual bool start() = 0;
			virtual void update() = 0;
			virtual void cleanup() = 0;
			virtual void stop() = 0;

			OutputPort<std::string> events;
			std::string m_events;
    };

	void check_event_edge(YouBotService* const serv, const motor_status ref_cond, const std::string outp_message, bool* const cond_state,
							unsigned int joint_id, motor_status current);

	void check_event_level(YouBotService* const serv, const motor_status ref_cond, const std::string outp_message,
							unsigned int joint_id, motor_status current);

}
