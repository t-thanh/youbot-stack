#include "YouBotBaseService.hpp"

#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>

#include "YouBotHelpers.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	extern unsigned int non_errors;

	YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent, unsigned int min_slave_nr) :
		Service(name,parent),
		m_tmp_joint_angles(NR_OF_BASE_SLAVES, JointSensedAngle(0*radian)),
		m_tmp_joint_velocities(NR_OF_BASE_SLAVES, JointSensedVelocity(0*radian_per_second)),
		m_tmp_joint_torques(NR_OF_BASE_SLAVES, JointSensedTorque(0*newton_meter)),

		m_joint_ctrl_modes(NR_OF_BASE_SLAVES, MOTOR_STOP),
		// Set the commands to zero depending on the number of joints
		m_calibrated(false),
		m_min_slave_nr(min_slave_nr)
	{
		m_joint_states.position.assign(NR_OF_BASE_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_BASE_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_BASE_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_BASE_SLAVES,0);
		m_joint_cmd_velocities.velocities.assign(NR_OF_BASE_SLAVES,0);
		m_joint_cmd_torques.efforts.assign(NR_OF_BASE_SLAVES,0);

		m_motor_statuses.flags.resize(NR_OF_BASE_SLAVES, 0);

		this->addPort("joint_states",joint_states).doc("Joint states");

		this->addPort("motor_statuses",motor_statuses).doc("Motor statuses");

		this->addPort("joint_cmd_angles",joint_cmd_angles).doc("Command joint angles");
		this->addPort("joint_cmd_velocities",joint_cmd_velocities).doc("Command joint velocities");
		this->addPort("joint_cmd_torques",joint_cmd_torques).doc("Command joint torques");

		this->addPort("joint_ctrl_modes",joint_ctrl_modes).doc("Joint controller modes");

		this->addOperation("start",&YouBotBaseService::start,this);
		this->addOperation("update",&YouBotBaseService::update,this);
		this->addOperation("calibrate",&YouBotBaseService::calibrate,this);
		this->addOperation("stop",&YouBotBaseService::stop,this);
		this->addOperation("cleanup",&YouBotBaseService::cleanup,this);

		this->addOperation("getBasePosition",&YouBotBaseService::getBasePosition,this, OwnThread);
		this->addOperation("setBasePosition",&YouBotBaseService::setBasePosition,this, OwnThread);
//
		this->addOperation("getBaseVelocity",&YouBotBaseService::getBaseVelocity,this, OwnThread);
		this->addOperation("setBaseVelocity",&YouBotBaseService::setBaseVelocity,this, OwnThread);

		this->addOperation("setControlModes",&YouBotBaseService::setControlModes,this, OwnThread);
		this->addOperation("getControlModes",&YouBotBaseService::getControlModes,this, OwnThread);
		this->addOperation("displayMotorStatuses",&YouBotBaseService::displayMotorStatuses,this, OwnThread);

		this->addOperation("check_error",&YouBotBaseService::check_error,this);

		// Pre-allocate port memory for outputs
        joint_states.setDataSample( m_joint_states );
        motor_statuses.setDataSample(m_motor_statuses);
	}

	YouBotBaseService::~YouBotBaseService()
	{
		delete m_base;
	}

	void YouBotBaseService::getBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)
	{
		log(Info) << "getBasePosition" << endlog();
		m_base->getBasePosition(longitudinalPosition, transversalPosition, orientation);
	}

	void YouBotBaseService::setBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)
	{
		log(Info) << "setBasePosition" << endlog();
		log(Info) << longitudinalPosition << " " << transversalPosition << " " << orientation << endlog();
		m_base->setBasePosition(longitudinalPosition, transversalPosition, orientation);
	}

	void YouBotBaseService::getBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity)
	{
		m_base->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
	}

	void YouBotBaseService::setBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity)
	{
		log(Info) << "setBaseVelocity" << endlog();
		log(Info) << longitudinalVelocity << " " << transversalVelocity << " " << angularVelocity << endlog();
		m_base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
		log(Info) << "done" << endlog();
	}

	void YouBotBaseService::setControlModes(vector<ctrl_modes>& all)
	{
//		log(Debug) << "Control modes set to: " << all << endlog();
		m_joint_ctrl_modes = all;
	}

	vector<ctrl_modes> YouBotBaseService::getControlModes()
	{
		return m_joint_ctrl_modes;
	}

	bool YouBotBaseService::start()
	{
		return m_calibrated;
	}

	void YouBotBaseService::updateJointSetpoint(unsigned int joint_nr)
	{
		assert(joint_nr < NR_OF_BASE_SLAVES);

		//@todo How to make sure that no OLD data is left behind in ports for next iterations?
		switch(m_joint_ctrl_modes[joint_nr])
		{
			case(PLANE_ANGLE):
			{
				m_tmp_joint_cmd_angle.angle = m_joint_cmd_angles.positions[joint_nr] * si::radian;
				m_joints[joint_nr]->setData(m_tmp_joint_cmd_angle);
				break;
			}
			case(ANGULAR_VELOCITY):
			{
				m_tmp_joint_cmd_velocity.angularVelocity = m_joint_cmd_velocities.velocities[joint_nr] * si::radian_per_second;
				m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
				break;
			}
			case(TORQUE):
			{
				m_tmp_joint_cmd_torque.torque = m_joint_cmd_torques.efforts[joint_nr] * si::newton_meter;
				m_joints[joint_nr]->setData(m_tmp_joint_cmd_torque);
				break;
			}
			case(MOTOR_STOP):
			{
				m_joints[joint_nr]->stopJoint();
				break;
			}
			default:
			{
				log(Error) << "Case not recognized." << endlog();
				break;
			}
		}
	}

	void YouBotBaseService::update()
	{
//		log(Info) << "YouBotBaseService update" << endlog();
		// YouBot -> OutputPort
		m_base->getJointData(m_tmp_joint_angles);
		m_base->getJointData(m_tmp_joint_velocities);
		m_base->getJointData(m_tmp_joint_torques);
//
		assert(m_tmp_joint_angles.size() == m_tmp_joint_velocities.size() && m_tmp_joint_velocities.size() == m_tmp_joint_torques.size());

		m_joint_states.header.stamp = ros::Time::now();

		int size = m_tmp_joint_angles.size();
		for(int i = 0; i < size; ++i)
		{
			m_joint_states.position[i] = m_tmp_joint_angles[i].angle.value();

			m_joint_states.velocity[i] = m_tmp_joint_velocities[i].angularVelocity.value();

			m_joint_states.effort[i] = m_tmp_joint_torques[i].torque.value();
		}

		joint_states.write(m_joint_states);

		// InputPort -> YouBot
		joint_ctrl_modes.read(m_joint_ctrl_modes);
		joint_cmd_angles.read(m_joint_cmd_angles);
		joint_cmd_velocities.read(m_joint_cmd_velocities);
		joint_cmd_torques.read(m_joint_cmd_torques);

		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			updateJointSetpoint(i);
		}

		// Check for errors:
		check_error();
	}

	bool YouBotBaseService::calibrate()
	{
		if(m_calibrated)
		{
			log(Info) << "Already calibrated." << endlog();
			return m_calibrated;
		}

		log(Info) << "Calibrating YouBotBaseService..." << endlog();
		try
		{
			m_base = new YouBotBase("/youbot-base", OODL_YOUBOT_CONFIG_DIR);
			if(m_base == NULL)
			{
				log(Error) << "Could not create the YouBotBase." << endlog();
				return false;
			}

			m_base->doJointCommutation();

			for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
			{
				//@todo Fixme: m_min_slave_nr
				m_joints[i] = &(m_base->getBaseJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
			}
		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			m_base = NULL;
			return false;
		}

		log(Info) << "Calibrated." << endlog();
		return (m_calibrated = true);
	}

	bool YouBotBaseService::check_error()
	{
		bool found_error(false);
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i]->getStatus(m_motor_statuses.flags[i]);
			if(m_motor_statuses.flags[i] != 0)
			{
				found_error = true;
			}
		}

		if(found_error)
		{
			motor_statuses.write(m_motor_statuses);
		}
		//emit errors via port.
		return found_error;
	}

	void YouBotBaseService::stop()
	{
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i]->stopJoint();
		}
	}

	void YouBotBaseService::cleanup()
	{
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i] = NULL;
		}
		delete m_base;
		m_base = NULL;
		m_calibrated = false;
	}

	void YouBotBaseService::displayMotorStatuses()
	{
		for(unsigned int i = 0; i < m_motor_statuses.flags.size(); ++i)
		{
			log(Info) << "Joint[" << i+1 << "] is " << motor_status_tostring(m_motor_statuses.flags[i]) << endlog();
		}
	}

}
