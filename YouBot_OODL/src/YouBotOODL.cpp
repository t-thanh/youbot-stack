#include "YouBotOODL.hpp"
#include "YouBotBaseService.hpp"
#include "YouBotArmService.hpp"
#include "YouBotHelpers.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <generic/Logger.hpp>
#include <rtt/Logger.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	YouBotOODL::YouBotOODL(const string& name) : TaskContext(name, PreOperational)
	{
		youbot::Logger::logginLevel = youbot::fatal;
		RTT::Logger* ins = RTT::Logger::Instance();
		ins->setLogLevel(RTT::Logger::Info);
	}

	YouBotOODL::~YouBotOODL() {}

	bool YouBotOODL::configureHook()
	{
		// MUST BE THE FIRST ONE TO CALL getInstance!!!
		unsigned int nr_slaves = 0;

		try
		{
			m_ec_master = &(EthercatMaster::getInstance("/youbot-ethercat.cfg", OODL_YOUBOT_CONFIG_DIR));

			nr_slaves = m_ec_master->getNumberOfSlaves();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			this->error();
			return false;
		}

		if(nr_slaves != (NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) && nr_slaves != (NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES))
		{
			log(Error) << "Not a proper amount of Ethercat slaves, got:" << nr_slaves << endlog();
			return false;
		}

		// Base
		this->provides()->addService(Service::shared_ptr( new YouBotBaseService("Base",this, 1) ) );
		update_ops.push_back(this->provides("Base")->getOperation("update"));
		calibrate_ops.push_back(this->provides("Base")->getOperation("calibrate"));
		start_ops.push_back(this->provides("Base")->getOperation("start"));
		stop_ops.push_back(this->provides("Base")->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Base")->getOperation("cleanup"));
		log(Info) << "Detected youbot base, loading Base service" << endlog();

		// Arm 1
		this->provides()->addService(Service::shared_ptr( new YouBotArmService("Manipulator1",this, 1) ) );
		update_ops.push_back(this->provides("Manipulator1")->getOperation("update"));
		calibrate_ops.push_back(this->provides("Manipulator1")->getOperation("calibrate"));
		start_ops.push_back(this->provides("Manipulator1")->getOperation("start"));
		stop_ops.push_back(this->provides("Manipulator1")->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Manipulator1")->getOperation("cleanup"));
		log(Info) << "Detected youbot arm, loading Manipulator1 service" << endlog();

		if(nr_slaves == NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES) // Arm 2
		{
			this->provides()->addService(Service::shared_ptr( new YouBotArmService("Manipulator2",this, 1 + NR_OF_ARM_SLAVES) ) );
			update_ops.push_back(this->provides("Manipulator2")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Manipulator2")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Manipulator2")->getOperation("start"));
			stop_ops.push_back(this->provides("Manipulator2")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Manipulator2")->getOperation("cleanup"));
			log(Info) << "Detected youbot arm, loading Manipulator2 service" << endlog();
		}

		Seconds period = this->getPeriod();
		if(period < 0.001)
		{
			log(Error) << "The EthercatMaster thread needs at least 1kHz frequency to operate properly." << endlog();
			return false;
		}

		log(Info) << "EthercatMaster thread period: " << period << endlog();

		// invoke all calibration operations
		bool proper_calibration(true);
		for(unsigned int i=0; i<calibrate_ops.size(); ++i)
		{
			if(calibrate_ops[i].ready())
			{
				if(!calibrate_ops[i]())
				{
					proper_calibration = false;
					break;
				}
			}
		}

		// Break off when a part of the robot couldn't calibrate.
		if(!proper_calibration)
		{
			cleanupHook();
			update_ops.clear();
			calibrate_ops.clear();
			start_ops.clear();
			stop_ops.clear();
			cleanup_ops.clear();
			return false;
		}
		else
		{
			return TaskContext::configureHook();
		}
	}

	bool YouBotOODL::startHook()
	{
		// invoke all starts
		bool fully_started(true);

        for(unsigned int i=0;i<start_ops.size();++i)
        {
            if(start_ops[i].ready())
            {
            	if(!start_ops[i]())
            	{
            		fully_started = false;
            		break;
            	}
            }
        }

        return fully_started ? TaskContext::startHook() : false;
	}

	void YouBotOODL::updateHook()
	{
		m_ec_master->sendAndReceiveProcessData();

		// The mailbox messages are send/received immediatly

        for(unsigned int i=0;i<update_ops.size();++i)
        {
            if(update_ops[i].ready())
            {
                update_ops[i]();
            }
        }

        TaskContext::updateHook();
	}

	void YouBotOODL::stopHook()
	{
        for(unsigned int i=0;i<stop_ops.size();++i)
        {
            if(stop_ops[i].ready())
            {
            	stop_ops[i]();
            }
        }

        TaskContext::stopHook();
	}

	void YouBotOODL::cleanupHook()
	{
        for(unsigned int i=0;i<cleanup_ops.size();++i)
        {
            if(cleanup_ops[i].ready())
            {
            	cleanup_ops[i]();
            }
        }

        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::YouBotOODL )
