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
#include <youbot/EthercatMasterWithoutThread.hpp>

enum arm_settings {NR_OF_ARM_SLAVES=5};
enum base_settings {NR_OF_BASE_SLAVES=4};

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	class YouBotOODL: public TaskContext
	{
		public:
			YouBotOODL(const string& name);
			virtual ~YouBotOODL();

			void emitEvent(std::string message);
			void emitEvent(unsigned int joint, std::string message);
			void emitEvent(unsigned int joint, std::string message, bool condition);

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

			OutputPort<YouBot_OODL::driver_event> events;

		private:
	        vector<OperationCaller<bool(void)> > calibrate_ops;
	        vector<OperationCaller<bool(void)> > start_ops;
	        vector<OperationCaller<void(void)> > update_ops;
	        vector<OperationCaller<void(void)> > stop_ops;
	        vector<OperationCaller<void(void)> > cleanup_ops;

	        youbot::EthercatMaster* m_ec_master;

	        YouBot_OODL::driver_event m_events;
	};

	void check_edge(YouBotOODL* oodl, const motor_status ref_cond, const std::string outp_message, bool* cond_states,
			unsigned int joint, motor_status current);

	void check_level(YouBotOODL* oodl, const motor_status ref_cond, const std::string outp_message,
				unsigned int joint, motor_status current);
}
