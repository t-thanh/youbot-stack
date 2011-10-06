#include "BaseControllerMockup.h"
#include "YouBotOODL.h"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace boost::units;
	using namespace boost::units::si;

	BaseControllerMockup::BaseControllerMockup(const string& name) :
			TaskContext(name, PreOperational),
//			m_joint_velocities(NR_OF_BASE_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_torques(NR_OF_BASE_SLAVES, quantity<si::torque>(0*newton_meter)),
			m_joint_statuses(NR_OF_BASE_SLAVES, 0)
//			m_modes(NR_OF_BASE_SLAVES, PLANE_ANGLE),
			// Set the commands to zero depending on the number of joints
//			m_joint_cmd_velocities(NR_OF_BASE_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_cmd_torques(NR_OF_BASE_SLAVES, quantity<si::torque>(0*newton_meter)),
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

		this->addOperation("setJointAngles",&BaseControllerMockup::setJointAngles,this, OwnThread);

		this->addOperation("getBasePosition",&BaseControllerMockup::getBasePosition,this, OwnThread);
		this->addOperation("setBasePosition",&BaseControllerMockup::setBasePosition,this, OwnThread);

		this->addOperation("setBaseVelocity",&BaseControllerMockup::setBaseVelocity,this, OwnThread);
	}

	BaseControllerMockup::~BaseControllerMockup() {}

	void BaseControllerMockup::setJointAngles(vector< double >& angles, double epsilon)
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

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

	void BaseControllerMockup::getBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation)
	{
		quantity<length> _longitudinalPosition;
		quantity<length> _transversalPosition;
		quantity<plane_angle> _orientation;

		op_getBasePosition(_longitudinalPosition, _transversalPosition, _orientation);
		longitudinalPosition = _longitudinalPosition.value();
		transversalPosition = _transversalPosition.value();
		orientation = _orientation.value() * 180 / M_PI;
	}

	void BaseControllerMockup::setBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation)
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		quantity<length> _longitudinalPosition = longitudinalPosition * si::meter;
		quantity<length> _transversalPosition = transversalPosition * si::meter;
		quantity<plane_angle> _orientation = orientation * M_PI / 180 * si::radian;
		op_setBasePosition(_longitudinalPosition, _transversalPosition, _orientation);
	}

	void BaseControllerMockup::getBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity)
	{
		log(Error) << "Not implemented." << endlog();
	}

	void BaseControllerMockup::setBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity)
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, ANGULAR_VELOCITY);

		quantity<velocity> _longitudinalVelocity = longitudinalVelocity * si::meter_per_second;
		quantity<velocity> _transversalVelocity = transversalVelocity * si::meter_per_second;
		quantity<angular_velocity> _angularVelocity = angularVelocity * M_PI / 180 * si::radian_per_second;
		op_setBaseVelocity(_longitudinalVelocity, _transversalVelocity, _angularVelocity);
	}

	bool BaseControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool BaseControllerMockup::startHook()
	{
		if(!joint_states.connected() || !joint_cmd_angles.connected() || !joint_statuses.connected() || !joint_ctrl_modes.connected())
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		TaskContext* task_ptr = getPeer("OODL");
		if(task_ptr == NULL)
		{
			log(Error) << "Could not find peer OODL" << endlog();
			return false;
		}
		op_getBasePosition = task_ptr->provides("Base")->getOperation("getBasePosition");
		op_setBasePosition = task_ptr->provides("Base")->getOperation("setBasePosition");

		op_setBaseVelocity = task_ptr->provides("Base")->getOperation("setBaseVelocity");
		return TaskContext::startHook();
	}

	void BaseControllerMockup::updateHook()
	{
        TaskContext::updateHook();

        joint_states.read(m_joint_states);
        joint_statuses.read(m_joint_statuses);

        joint_ctrl_modes.write(m_modes);
        joint_cmd_angles.write(m_joint_cmd_angles);

	}

	void BaseControllerMockup::stopHook()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, MOTOR_STOP);
		joint_ctrl_modes.write(m_modes);
        TaskContext::stopHook();
	}

	void BaseControllerMockup::cleanupHook()
	{
        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::BaseControllerMockup )
