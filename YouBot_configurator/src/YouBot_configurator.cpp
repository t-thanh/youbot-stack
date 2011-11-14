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

using namespace RTT;
using namespace std;
using namespace youbot;

namespace YouBot
{

YouBot_configurator::YouBot_configurator(const string& name) :
	TaskContext(name)
{ }

YouBot_configurator::~YouBot_configurator()
{ }

bool YouBot_configurator::configureHook()
{
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

	for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
	{
		stringstream name("arm-");
		name << i;
		name << "-parameter.cfg";

		stringstream protected_name("protected-arm-");
		protected_name << i;
		protected_name << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_manipulator->getArmJoint(i)), "../config/arm", name.str(), protected_name.str());
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
		stringstream name("base-");
		name << i;
		name << "-parameter.cfg";

		stringstream protected_name("protected-base-");
		protected_name << i;
		protected_name << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_base->getBaseJoint(i)), "../config/base", name.str(), protected_name.str());
			jc.readParameters();
			jc.setParametersToJoint();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
		}
	}

	return true;
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
