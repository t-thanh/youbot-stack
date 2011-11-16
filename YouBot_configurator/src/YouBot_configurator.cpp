#include "YouBot_configurator.hpp"

#include <stdlib.h>
#include <stdio.h>

//#include <cmath>
//#include <limits>
//#include <iostream>
//#include <vector>
//#include <signal.h>
//#include "youbot/YouBotJointParameter.hpp"
//#include "youbot/YouBotGripper.hpp"
//#include "boost/date_time/posix_time/posix_time.hpp"
//#include <vector>
//#include <sstream>
//#include <boost/limits.hpp>
//#include "generic/Logger.hpp"
//#include "generic/Units.hpp"
//#include "generic/Time.hpp"
//#include "generic/Exceptions.hpp"
//#include "generic-joint/JointParameter.hpp"
//#include "youbot/YouBotJointParameterReadOnly.hpp"

#include <YouBotTypes.hpp>
#include <JointConfigurator.hpp>

#include <ros/package.h>

using namespace RTT;
using namespace std;
using namespace youbot;

namespace YouBot
{

YouBot_configurator::YouBot_configurator(const string& name) :
	TaskContext(name, PreOperational), m_manipulator(NULL), m_base(NULL)
{
}

YouBot_configurator::~YouBot_configurator()
{
}

bool YouBot_configurator::configureHook()
{
	// MUST BE THE FIRST ONE TO CALL getInstance!!!
	unsigned int nr_slaves = 0;

	try
	{
		EthercatMaster* ec_master = &(EthercatMaster::getInstance("/youbot-ethercat.cfg", OODL_YOUBOT_CONFIG_DIR));

		nr_slaves = ec_master->getNumberOfSlaves();
	}
	catch (std::exception& e)
	{
		log(Error) << e.what() << endlog();
		this->error();
		return false;
	}

	if(nr_slaves != (NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) && nr_slaves != (NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES))
	{
		log(Error) << "Not a proper amount of Ethercat slaves, got:" << nr_slaves << endlog();
		return false;
	}

	try
	{
		m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
		if(m_manipulator == NULL)
		{
			log(Error) << "Could not create the YouBotManipulator." << endlog();
			return false;
		}

		m_base = new YouBotBase("/youbot-base", OODL_YOUBOT_CONFIG_DIR);
		if(m_base == NULL)
		{
			log(Error) << "Could not create the YouBotBase." << endlog();
			return false;
		}
	}
	catch (std::exception& e)
	{
		log(Error) << e.what();
		m_manipulator = NULL;
		this->error();
		return false;
	}

	string path = ros::package::getPath("YouBot_configurator");
	stringstream arm_path;
	arm_path << path << "/config/arm";
	stringstream base_path;
	base_path << path << "/config/base";

	for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
	{
		stringstream name;
		name << "/arm-" << i << "-parameter.cfg";

		stringstream protected_name;
		protected_name << "/protected-arm-" << i << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_manipulator->getArmJoint(i)), arm_path.str(), name.str(), protected_name.str());
			jc.readParameters();
			jc.setParametersToJoint();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
		}
	}

	for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
	{
		stringstream name;
		name << "/base-" << i << "-parameter.cfg";

		stringstream protected_name;
		protected_name << "/protected-base-" << i << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_base->getBaseJoint(i)), base_path.str(), name.str(), protected_name.str());
			jc.readParameters();
			jc.setParametersToJoint();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
		}
	}

	return TaskContext::configureHook();
}

bool YouBot_configurator::startHook()
{
	log(Error) << "The configurator component is not runnable." << endlog();
	return false;
}

void YouBot_configurator::updateHook()
{
	//empty
}


}

ORO_CREATE_COMPONENT( YouBot::YouBot_configurator)
