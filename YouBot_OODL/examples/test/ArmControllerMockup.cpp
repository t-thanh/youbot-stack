#include "ArmControllerMockup.h"
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

	const double jointsInitialPosition[] = { 169 * M_PI / 180.0, 65 * M_PI / 180.0,
			-146 * M_PI / 180.0, 102.5 * M_PI / 180.0, 167.5 * M_PI / 180.0 }; //radian!

	ArmControllerMockup::ArmControllerMockup(const string& name) :
			TaskContext(name, PreOperational),
//			m_joint_velocities(NR_OF_ARM_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_torques(NR_OF_ARM_SLAVES, quantity<si::torque>(0*newton_meter)),
			m_joint_statuses(NR_OF_ARM_SLAVES, 0),
			m_modes(NR_OF_ARM_SLAVES, MOTOR_STOP)
			// Set the commands to zero depending on the number of joints
//			m_joint_cmd_velocities(NR_OF_ARM_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_cmd_torques(NR_OF_ARM_SLAVES, quantity<si::torque>(0*newton_meter)),
	{
		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_ARM_SLAVES,0);

		this->addPort("joint_states" , joint_states).doc("Input joint states from the driver");
		this->addPort("joint_cmd_angles", joint_cmd_angles).doc("Output joint angle commands to the driver");
		this->addPort("joint_statuses", joint_statuses).doc("Input joint statuses from the driver");
		this->addPort("joint_ctrl_modes", joint_ctrl_modes).doc("Joint control modes.");

		joint_cmd_angles.setDataSample(m_joint_cmd_angles);

		this->addOperation("setJointAngles",&ArmControllerMockup::setJointAngles,this, OwnThread).doc("In degrees");
		this->addOperation("getJointAngles",&ArmControllerMockup::getJointAngles,this, OwnThread).doc("In degrees");
		this->addOperation("unfoldManipulator",&ArmControllerMockup::unfoldManipulator,this, OwnThread);

		RTT::types::Types()->addType( new RTT::types::SequenceTypeInfo<std::vector<double> >("std.vector<double>") );
	}

	ArmControllerMockup::~ArmControllerMockup() {}

	void ArmControllerMockup::setJointAngles(vector< double >& angles, double epsilon)
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		// convert degrees to radian
		for(unsigned int  i = 0; i < angles.size(); ++i)
		{
			m_joint_cmd_angles.positions[i] = angles[i] * M_PI / 180; //to radian
		}

		joint_ctrl_modes.write(m_modes);
        joint_cmd_angles.write(m_joint_cmd_angles);

		bool done = false;
		while(!done)
		{
			joint_states.read(m_joint_states);

			done = true;
			for(unsigned int i = 0; i < angles.size(); ++i)
			{
				if( abs(m_joint_states.position[i] * 180 / M_PI - angles[i]) > epsilon) //in degrees
				{
					done = false;
					break;
				}
			}
		}
	}

	bool ArmControllerMockup::getJointAngles(vector< double >& angles)
	{
		if(angles.size() == m_joint_states.position.size())
		{
			// convert degrees to radian
			for(unsigned int  i = 0; i < angles.size(); ++i)
			{
				angles[i] = m_joint_states.position[i] * 180 / M_PI;
			}
			return true;
		}
		else
		{
			log(Error) << "Array size mismatch." << endlog();
			return false;
		}
	}

	void ArmControllerMockup::unfoldManipulator()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joint_cmd_angles.positions[i] = jointsInitialPosition[i];
		}
	}

	bool ArmControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool ArmControllerMockup::startHook()
	{
		if(!joint_states.connected() || !joint_cmd_angles.connected() || !joint_statuses.connected() || !joint_ctrl_modes.connected())
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void ArmControllerMockup::updateHook()
	{
        TaskContext::updateHook();

        joint_states.read(m_joint_states);
        joint_statuses.read(m_joint_statuses);

        joint_ctrl_modes.write(m_modes);
        joint_cmd_angles.write(m_joint_cmd_angles);

	}

	void ArmControllerMockup::stopHook()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, MOTOR_STOP);
		joint_ctrl_modes.write(m_modes);

        TaskContext::stopHook();
	}

	void ArmControllerMockup::cleanupHook()
	{
        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::ArmControllerMockup )
