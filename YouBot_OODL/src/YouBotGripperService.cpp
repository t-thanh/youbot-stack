#include "YouBotGripperService.hpp"

#include <stdio.h>
#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>

#include "YouBotHelpers.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	YouBotGripperService::YouBotGripperService(const string& name, TaskContext* parent, unsigned int min_slave_nr) :
			Service(name, parent),
//			m_joint_states(NR_OF_ARM_SLAVES),

			m_tmp_joint_angles(NR_OF_ARM_SLAVES, JointSensedAngle(0*radian)),
			m_tmp_joint_velocities(NR_OF_ARM_SLAVES, JointSensedVelocity(0*radian_per_second)),
			m_tmp_joint_torques(NR_OF_ARM_SLAVES, JointSensedTorque(0*newton_meter)),

			m_joint_limits(NR_OF_ARM_SLAVES),
			m_joint_statuses(NR_OF_ARM_SLAVES, 0),
			m_joint_ctrl_modes(NR_OF_ARM_SLAVES, MOTOR_STOP),

			// Set the commands to zero depending on the number of joints
			m_calibrated(false),
			m_min_slave_nr(min_slave_nr)
	{
		m_ec_master = EthercatMaster::getInstance();

		m_joint_states.name.assign(NR_OF_ARM_SLAVES,"");
		m_joint_states.name[0]="arm_joint_1";
		m_joint_states.name[1]="arm_joint_2";
		m_joint_states.name[2]="arm_joint_3";
		m_joint_states.name[3]="arm_joint_4";
		m_joint_states.name[4]="arm_joint_5";

//		m_gripper_state.name.assign(1, "");
//		m_gripper_state.name[0] = "gripper";

		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

//		m_gripper_state.position.assign(1, 0);
//		m_gripper_state.velocity.assign(0);
//		m_gripper_state.effort.assign(0);

		m_joint_cmd_angles.positions.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_velocities.velocities.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_torques.efforts.assign(NR_OF_ARM_SLAVES,0);

		m_gripper_cmd_position.positions.assign(1, 0);

		this->addPort("joint_states",joint_states).doc("Joint states");

		this->addPort("joint_statuses",joint_statuses).doc("Joint statuses");

		this->addPort("joint_cmd_angles",joint_cmd_angles).doc("Command joint angles");
		this->addPort("joint_cmd_velocities",joint_cmd_velocities).doc("Command joint velocities");
		this->addPort("joint_cmd_torques",joint_cmd_torques).doc("Command joint torques");

		this->addPort("gripper_cmd_position", gripper_cmd_position).doc("Command the gripper position");

		this->addPort("joint_ctrl_modes",joint_ctrl_modes).doc("Joint controller modes");

        this->addOperation("start",&YouBotGripperService::start,this);
        this->addOperation("update",&YouBotGripperService::update,this);
        this->addOperation("calibrate",&YouBotGripperService::calibrate,this);
        this->addOperation("stop",&YouBotGripperService::stop,this);
        this->addOperation("cleanup",&YouBotGripperService::cleanup,this);

        this->addOperation("setControlModes",&YouBotGripperService::setControlModes,this, OwnThread);
        this->addOperation("getControlModes",&YouBotGripperService::getControlModes,this, OwnThread);
        this->addOperation("displayJointStatuses",&YouBotGripperService::displayJointStatuses,this, OwnThread);

        this->addOperation("check_error",&YouBotGripperService::check_error,this, OwnThread);

        // Pre-allocate port memory for outputs
        joint_states.setDataSample( m_joint_states );
//        gripper_state.setDataSample(m_gripper_state);
        joint_statuses.setDataSample(m_joint_statuses);
	}

	YouBotGripperService::~YouBotGripperService()
	{
		delete m_manipulator;
	}

	void YouBotGripperService::setControlModes(vector<ctrl_modes>& all)
	{
//		log(Debug) << "Control modes set to: " << all << endlog();
		m_joint_ctrl_modes = all;
	}

	void YouBotGripperService::getControlModes(vector<ctrl_modes>& all)
	{
		log(Info) << "getting the control modes" << endlog();
		all = m_joint_ctrl_modes;
	}

	void YouBotGripperService::displayJointStatuses()
	{
		for(unsigned int i = 0; i < m_joint_statuses.size(); ++i)
		{
			log(Info) << "Joint[" << i+1 << "] is " << joint_status_tostring(m_joint_statuses[i]) << endlog();
		}
	}

	bool YouBotGripperService::start()
	{
		return m_calibrated;
	}

	void YouBotGripperService::updateJointSetpoint(unsigned int joint_nr)
	{
		assert(joint_nr < NR_OF_ARM_SLAVES);

		switch(m_joint_ctrl_modes[joint_nr])
		{
			case(PLANE_ANGLE):
			{
				m_tmp_joint_cmd_angle.angle = m_joint_cmd_angles.positions[joint_nr] * si::radian;
				// below limits
				if(m_tmp_joint_cmd_angle.angle < m_joint_limits[joint_nr].min_angle)
				{
					m_tmp_joint_cmd_angle.angle = m_joint_limits[joint_nr].min_angle;
				}
				// above limits
				else if(m_tmp_joint_cmd_angle.angle > m_joint_limits[joint_nr].max_angle)
				{
					m_tmp_joint_cmd_angle.angle = m_joint_limits[joint_nr].max_angle;
				}
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

	void YouBotGripperService::update()
	{
		ros::Time time = ros::Time::now();

		// YouBot -> OutputPort
		m_manipulator->getJointData(m_tmp_joint_angles);
		m_manipulator->getJointData(m_tmp_joint_velocities);
		m_manipulator->getJointData(m_tmp_joint_torques);

		// The OODL gripper does not support this.
//		m_gripper->getData(m_tmp_gripper_state);

		assert(m_tmp_joint_angles.size() == m_tmp_joint_velocities.size() && m_tmp_joint_velocities.size() == m_tmp_joint_torques.size());

		m_joint_states.header.stamp = time;

		int size = m_tmp_joint_angles.size();
		for(int i = 0; i < size; ++i)
		{
			m_joint_states.position[i] = m_tmp_joint_angles[i].angle.value();

			m_joint_states.velocity[i] = m_tmp_joint_velocities[i].angularVelocity.value();

			m_joint_states.effort[i] = m_tmp_joint_torques[i].torque.value();
		}

		joint_states.write(m_joint_states);

//		m_gripper_state.header.stamp = m_joint_states.header.stamp;
//		m_gripper_state.position[0] = m_tmp_gripper_state.barSpacing.value();

//		gripper_state.write(m_gripper_state);

		// InputPort -> YouBot
		joint_ctrl_modes.read(m_joint_ctrl_modes);
		joint_cmd_angles.read(m_joint_cmd_angles);
		joint_cmd_velocities.read(m_joint_cmd_velocities);
		joint_cmd_torques.read(m_joint_cmd_torques);

		// Update joint setpoints
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			updateJointSetpoint(i);
		}

		// Update gripper setpoint
		// TODO: Wait until the gripper OODL code is fixed.
//		if(gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
//		{
//			m_tmp_gripper_cmd_position.barSpacing = m_gripper_cmd_position.positions[0] * si::meter;
//			// check limits to prevent exceptions
//			if( m_tmp_gripper_cmd_position.barSpacing < m_gripper_limits.min_position )
//			{
//				m_tmp_gripper_cmd_position.barSpacing = m_gripper_limits.min_position;
//			}
//			//above limits:
//			else if(m_tmp_gripper_cmd_position.barSpacing > m_gripper_limits.max_position)
//			{
//				m_tmp_gripper_cmd_position.barSpacing = m_gripper_limits.max_position;
//			}
//
//			m_gripper->setData(m_tmp_gripper_cmd_position);
//		}

		// Check for errors:
		check_error();
	}

	bool YouBotGripperService::calibrate()
	{
		log(Info) << "Calibrating YouBotGripperService" << endlog();
		if(m_calibrated)
		{
			log(Info) << "Already calibrated." << endlog();
			return m_calibrated;
		}

		//@todo What about 2 arms?
		try
		{
			m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
			if(m_manipulator == NULL)
			{
				log(Error) << "Could not create the YouBotManipulator." << endlog();
				return false;
			}

			m_manipulator->doJointCommutation();
			m_manipulator->calibrateManipulator();

			for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
			{
				//@todo Fixme: m_min_slave_nr
				m_joints[i] = &(m_manipulator->getArmJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
//				m_joint_cmd =
			}

			// Determine JointLimit's -> WORKAROUND to prevent exceptions!
			for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
			{
				// position limits
				youbot::JointLimits lim;
				m_joints[i]->getConfigurationParameter(lim);
				int lower_limit, upper_limit;
				bool limits_active;
				lim.getParameter(lower_limit, upper_limit, limits_active);
				if(!limits_active)
				{
					log(Error) << "JointLimits are not active, cannot function like this." << endlog();
					return false;
				}
				EncoderTicksPerRound enc;
				m_joints[i]->getConfigurationParameter(enc);
				unsigned int ticks_per_round(0);
				enc.getParameter(ticks_per_round);
				GearRatio gRatio;
				double gearRatio;
				m_joints[i]->getConfigurationParameter(gRatio);
				gRatio.getParameter(gearRatio);
				m_joint_limits[i].min_angle = ((double) lower_limit / ticks_per_round) * gearRatio * (2.0 * M_PI) * radian;
				m_joint_limits[i].max_angle = ((double) upper_limit / ticks_per_round) * gearRatio * (2.0 * M_PI) * radian;

				InverseMovementDirection invMov;
				m_joints[i]->getConfigurationParameter(invMov);
				bool invMov2(false);
				invMov.getParameter(invMov2);
				if(invMov2) //@todo: strange!!
				{
					quantity<plane_angle> tmp = m_joint_limits[i].min_angle;
					m_joint_limits[i].min_angle = -m_joint_limits[i].max_angle;
					m_joint_limits[i].max_angle = -tmp;
				}

				// OODL uses value < max/min instead of <=
				m_joint_limits[i].min_angle *= 0.999;
				m_joint_limits[i].max_angle *= 1.001;

				log(Info) << "Min angle: " << m_joint_limits[i].min_angle << " Max angle: " << m_joint_limits[i].max_angle << endlog();

				// velocity limits
				// There are no velocity limits at the moment!

				// torque limits
				// There are no current limits at the moment!


			}

			// Gripper
			m_manipulator->calibrateGripper();
			m_gripper = &(m_manipulator->getArmGripper());

			// Determine gripper limits to prevent exceptions
			MaxTravelDistance _max_distance;
			BarSpacingOffset _spacing;
			quantity<length> max_distance;
			quantity<length> spacing;
			m_gripper->getConfigurationParameter(_max_distance, BAR_ONE);
			m_gripper->getConfigurationParameter(_spacing, BAR_ONE);
			_max_distance.getParameter(max_distance);
			_spacing.getParameter(spacing);
			m_gripper_limits.min_position = spacing;
			m_gripper_limits.max_position = max_distance + spacing;

		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			m_manipulator = NULL;
			return false;
		}

		return (m_calibrated = true);
	}

	bool YouBotGripperService::check_error()
	{
		bool found_error(false);
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->getStatus(m_joint_statuses[i]);
			m_joint_statuses[i] = m_joint_statuses[i] & !non_errors; //filter non errors!
			//@todo: Check if initialized!
			if( m_joint_statuses[i] != 0)
			{
				found_error = true;
			}
		}

		if(found_error)
		{
			joint_statuses.write(m_joint_statuses);
		}
		//emit errors via port.
		return found_error;
	}

	void YouBotGripperService::stop()
	{
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->stopJoint();
		}
	}

	void YouBotGripperService::cleanup()
	{
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i] = NULL;
		}
		delete m_manipulator;
		m_manipulator = NULL;
		m_calibrated = false;
	}

}
