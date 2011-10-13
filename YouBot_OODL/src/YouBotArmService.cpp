#include "YouBotArmService.hpp"

#include <stdio.h>
#include <cassert>

#include "YouBotHelpers.hpp"
#include <youbot/ProtocolDefinitions.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	extern unsigned int non_errors;

	YouBotArmService::YouBotArmService(const string& name, TaskContext* parent, unsigned int min_slave_nr) :
			Service(name, parent),

			m_tmp_joint_angles(NR_OF_ARM_SLAVES, JointSensedAngle(0*radian)),
			m_tmp_joint_velocities(NR_OF_ARM_SLAVES, JointSensedVelocity(0*radian_per_second)),
			m_tmp_joint_torques(NR_OF_ARM_SLAVES, JointSensedTorque(0*newton_meter)),

			m_joint_limits(NR_OF_ARM_SLAVES),
			m_joint_ctrl_modes(NR_OF_ARM_SLAVES, MOTOR_STOP),

			// Set the commands to zero depending on the number of joints
			m_calibrated(false),
			m_min_slave_nr(min_slave_nr)
	{
		m_joint_states.name.assign(NR_OF_ARM_SLAVES,"");
		m_joint_states.name[0]="arm_joint_1";
		m_joint_states.name[1]="arm_joint_2";
		m_joint_states.name[2]="arm_joint_3";
		m_joint_states.name[3]="arm_joint_4";
		m_joint_states.name[4]="arm_joint_5";

		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_velocities.velocities.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_torques.efforts.assign(NR_OF_ARM_SLAVES,0);

		m_motor_statuses.flags.resize(NR_OF_ARM_SLAVES, 0);
		m_events.driver_event.assign(50, ' '); //@TODO: Fix me

		this->addPort("joint_states",joint_states).doc("Joint states");
		this->addPort("motor_statuses",motor_statuses).doc("Motor statuses");
		this->addPort("events", events).doc("Joint events");

		this->addPort("joint_cmd_angles",joint_cmd_angles).doc("Command joint angles");
		this->addPort("joint_cmd_velocities",joint_cmd_velocities).doc("Command joint velocities");
		this->addPort("joint_cmd_torques",joint_cmd_torques).doc("Command joint torques");

		this->addPort("joint_ctrl_modes",joint_ctrl_modes).doc("Joint controller modes");

        this->addOperation("start",&YouBotArmService::start,this);
        this->addOperation("update",&YouBotArmService::update,this);
        this->addOperation("calibrate",&YouBotArmService::calibrate,this);
        this->addOperation("stop",&YouBotArmService::stop,this);
        this->addOperation("cleanup",&YouBotArmService::cleanup,this);

        this->addOperation("setControlModes",&YouBotArmService::setControlModes,this, OwnThread);
        this->addOperation("getControlModes",&YouBotArmService::getControlModes,this, OwnThread);
        this->addOperation("displayMotorStatuses",&YouBotArmService::displayMotorStatuses,this, OwnThread);

        // Pre-allocate port memory for outputs
        joint_states.setDataSample( m_joint_states );
        motor_statuses.setDataSample(m_motor_statuses);

        YouBotOODL* oodl = dynamic_cast<YouBotOODL*>(parent);

        // edge events
        check_fp cond = boost::bind(&check_edge, oodl, ::OVER_CURRENT, "e_OVERCURRENT", m_overcurrent, _1, _2);
        m_event_checks.push_back(cond);
        cond = boost::bind(&check_edge, oodl, ::UNDER_VOLTAGE, "e_UNDERVOLTAGE", m_undervoltage, _1, _2);
		m_event_checks.push_back(cond);

		// level events
		cond = boost::bind(&check_level, oodl, ::TIMEOUT, "e_EC_TIMEOUT", _1, _2);
		m_event_checks.push_back(cond);
	}

	YouBotArmService::~YouBotArmService()
	{
		delete m_manipulator;
	}

	void YouBotArmService::setControlModes(vector<ctrl_modes>& all)
	{
		m_joint_ctrl_modes = all;
	}

	void YouBotArmService::getControlModes(vector<ctrl_modes>& all)
	{
		all = m_joint_ctrl_modes;
	}

	void YouBotArmService::displayMotorStatuses()
	{
		for(unsigned int i = 0; i < m_motor_statuses.flags.size(); ++i)
		{
			log(Info) << "Joint[" << i+1 << "] is " << motor_status_tostring(m_motor_statuses.flags[i]) << endlog();
		}
	}

	bool YouBotArmService::start()
	{
		return m_calibrated;
	}

	void YouBotArmService::readJointStates()
	{
		ros::Time time = ros::Time::now();

		// YouBot -> OutputPort
		m_manipulator->getJointData(m_tmp_joint_angles);
		m_manipulator->getJointData(m_tmp_joint_velocities);
		m_manipulator->getJointData(m_tmp_joint_torques);

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
	}

	void YouBotArmService::updateJointSetpoints()
	{
		// InputPort -> YouBot
		joint_ctrl_modes.read(m_joint_ctrl_modes);
		joint_cmd_angles.read(m_joint_cmd_angles);
		joint_cmd_velocities.read(m_joint_cmd_velocities);
		joint_cmd_torques.read(m_joint_cmd_torques);

		// Update joint setpoints
		for(unsigned int joint_nr = 0; joint_nr < NR_OF_ARM_SLAVES; ++joint_nr)
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
	}

	void YouBotArmService::checkForErrors()
	{
		motor_status tmp(0);
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->getStatus(m_motor_statuses.flags[i]);

			tmp = m_motor_statuses.flags[i] & !non_errors; //filter non errors!

			for(unsigned int j = 0; i < m_event_checks.size(); ++j)
			{
				m_event_checks[j](i, tmp);
			}
		}

		motor_statuses.write(m_motor_statuses);
	}

	void YouBotArmService::update()
	{
		readJointStates();

		updateJointSetpoints();

		checkForErrors();
	}

	bool YouBotArmService::calibrate()
	{
		log(Info) << "Calibrating YouBotArmService" << endlog();
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

		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			m_manipulator = NULL;
			return false;
		}

		return (m_calibrated = true);
	}

	void YouBotArmService::stop()
	{
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->stopJoint();
		}
	}

	void YouBotArmService::cleanup()
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
