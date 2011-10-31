#include "GripperControllerMockup.h"
#include "YouBotOODL.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <boost/units/systems/si.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace boost::units;
	using namespace boost::units::si;

	GripperControllerMockup::GripperControllerMockup(const string& name) :
			TaskContext(name, PreOperational)
	{
//		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
//		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
//		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_gripper_cmd_position.positions.assign(1,0);

//		this->addPort("joint_states" , joint_states).doc("Input joint states from the driver");
		this->addPort("gripper_cmd_position", gripper_cmd_position).doc("Output gripper position.");

		gripper_cmd_position.setDataSample(m_gripper_cmd_position);

		this->addOperation("openGripper",&GripperControllerMockup::openGripper,this, OwnThread);
		this->addOperation("closeGripper",&GripperControllerMockup::closeGripper,this, OwnThread);
	}

	GripperControllerMockup::~GripperControllerMockup() {}

	void GripperControllerMockup::openGripper()
	{
		m_gripper_cmd_position.positions[0] = 0.01;
		gripper_cmd_position.write(m_gripper_cmd_position);
	}

	void GripperControllerMockup::closeGripper()
	{
		m_gripper_cmd_position.positions[0] = 0.0;
		gripper_cmd_position.write(m_gripper_cmd_position);
	}

	bool GripperControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool GripperControllerMockup::startHook()
	{
		if(!gripper_cmd_position.connected())
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void GripperControllerMockup::updateHook()
	{
        TaskContext::updateHook();
	}

	void GripperControllerMockup::stopHook()
	{
        TaskContext::stopHook();
	}

	void GripperControllerMockup::cleanupHook()
	{
        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::GripperControllerMockup )
