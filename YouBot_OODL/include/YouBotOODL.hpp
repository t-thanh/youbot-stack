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

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
	        vector<OperationCaller<bool(void)> > calibrate_ops;
	        vector<OperationCaller<bool(void)> > start_ops;
	        vector<OperationCaller<void(void)> > update_ops;
	        vector<OperationCaller<void(void)> > stop_ops;
	        vector<OperationCaller<void(void)> > cleanup_ops;

	        youbot::EthercatMaster* m_ec_master;

	        unsigned int m_communication_errors;
	        unsigned int m_max_communication_errors;
	};

}
