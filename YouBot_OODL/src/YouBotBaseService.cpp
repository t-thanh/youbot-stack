#include "YouBotBaseService.hpp"

#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>
#include <base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp>

#include "YouBotHelpers.hpp"

#include <tf/tf.h>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	extern unsigned int non_errors;

	YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent, unsigned int min_slave_nr) :
		YouBotService(name,parent),
		m_tmp_joint_angles(NR_OF_BASE_SLAVES, JointSensedAngle(0*radian)),
		m_tmp_joint_velocities(NR_OF_BASE_SLAVES, JointSensedVelocity(0*radian_per_second)),
		m_tmp_joint_torques(NR_OF_BASE_SLAVES, JointSensedTorque(0*newton_meter)),

		m_joint_ctrl_modes(NR_OF_BASE_SLAVES, MOTOR_STOP),
		// Set the commands to zero depending on the number of joints
		m_torque_offset(0.0),
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

		// Pre-allocate port memory for outputs
        joint_states.setDataSample( m_joint_states );
        motor_statuses.setDataSample(m_motor_statuses);
        odometry_state.setDataSample(m_odometry_state);

        // odometry pose estimates frame
        m_odometry_state.header.frame_id = "odometry";
        m_odometry_state.header.seq = 0;
        // odometry twist estimates frame
        m_odometry_state.child_frame_id = "base_link";

        // odometry estimates - set to zero
        m_odometry_state.pose.pose.position.x = 0;
        m_odometry_state.pose.pose.position.y = 0;
        m_odometry_state.pose.pose.position.z = 0;
        m_odometry_state.pose.pose.orientation.x = 0;
        m_odometry_state.pose.pose.orientation.y = 0;
        m_odometry_state.pose.pose.orientation.z = 0;
        m_odometry_state.pose.pose.orientation.w = 0;
        m_odometry_state.twist.twist.linear.x = 0;
        m_odometry_state.twist.twist.linear.y = 0;
        m_odometry_state.twist.twist.linear.z = 0;
        m_odometry_state.twist.twist.angular.x = 0;
        m_odometry_state.twist.twist.angular.y = 0;
        m_odometry_state.twist.twist.angular.z = 0;

		memset(m_overcurrent, 0, NR_OF_BASE_SLAVES); // set to false
		memset(m_undervoltage, 0, NR_OF_BASE_SLAVES);
		memset(m_overvoltage, 0, NR_OF_BASE_SLAVES);
		memset(m_overtemperature, 0, NR_OF_BASE_SLAVES);
		memset(m_connectionlost, 0, NR_OF_BASE_SLAVES);
		memset(m_i2texceeded, 0, NR_OF_BASE_SLAVES);
		memset(m_timeout, 0, NR_OF_BASE_SLAVES);

		m_torque_offset.resize(NR_OF_BASE_SLAVES, 0.0);

        setupComponentInterface();
        setupEventChecks();
	}

	YouBotBaseService::~YouBotBaseService()
	{
		delete m_base;
	}

	void YouBotBaseService::setupComponentInterface()
	{
		YouBotService::setupComponentInterface();

		this->addPort("joint_states",joint_states).doc("Joint states");
		this->addPort("odometry_state",odometry_state).doc("Base odometry");

		this->addPort("motor_statuses",motor_statuses).doc("Motor statuses");

		this->addPort("joint_cmd_angles",joint_cmd_angles).doc("Command joint angles");
		this->addPort("joint_cmd_velocities",joint_cmd_velocities).doc("Command joint velocities");
		this->addPort("joint_cmd_torques",joint_cmd_torques).doc("Command joint torques");

		this->addPort("cmd_twist",cmd_twist).doc("Command base twist");

		this->addProperty("torque_offset", m_torque_offset).doc("Currently used torque offset");

		this->addOperation("start",&YouBotBaseService::start,this);
		this->addOperation("update",&YouBotBaseService::update,this);
		this->addOperation("calibrate",&YouBotBaseService::calibrate,this);
		this->addOperation("stop",&YouBotBaseService::stop,this);
		this->addOperation("cleanup",&YouBotBaseService::cleanup,this);

		this->addOperation("setControlModes",&YouBotBaseService::setControlModes,this, OwnThread);
		this->addOperation("getControlModes",&YouBotBaseService::getControlModes,this, OwnThread);

		this->addOperation("displayMotorStatuses",&YouBotBaseService::displayMotorStatuses,this, OwnThread);
		this->addOperation("clearControllerTimeouts",&YouBotBaseService::clearControllerTimeouts,this, OwnThread);
		this->addOperation("calibrateTorqueOffset",&YouBotBaseService::calibrateTorqueOffset,this, OwnThread);
	}

	void YouBotBaseService::setupEventChecks()
	{
		// edge events
		check_fp cond(NULL);
		cond = boost::bind(&check_event_edge, this, ::OVER_CURRENT, E_OVERCURRENT, m_overcurrent, _1, _2);
		m_event_checks.push_back(cond);

		cond = boost::bind(&check_event_edge, this, ::UNDER_VOLTAGE, E_UNDERVOLTAGE, m_undervoltage, _1, _2);
		m_event_checks.push_back(cond);

		cond = boost::bind(&check_event_edge, this, ::OVER_VOLTAGE, E_OVERVOLTAGE, m_overvoltage, _1, _2);
		m_event_checks.push_back(cond);

		cond = boost::bind(&check_event_edge, this, ::OVER_TEMPERATURE, E_OVERTEMP, m_overtemperature, _1, _2);
		m_event_checks.push_back(cond);

	//		cond = boost::bind(&check_event_edge, this, , E_EC_CON_LOST, m_connectionlost, _1, _2);
	//		m_event_checks.push_back(cond);

		cond = boost::bind(&check_event_edge, this, ::I2T_EXCEEDED, E_I2T_EXCEEDED, m_i2texceeded, _1, _2);
		m_event_checks.push_back(cond);


		// level events
		cond = boost::bind(&check_event_level, this, ::HALL_SENSOR_ERROR, E_HALL_ERR, _1, _2);
		m_event_checks.push_back(cond);

	//		cond = boost::bind(&check_event_level, this, ::ENCODER_ERROR, E_ENCODER_ERR, _1, _2);
	//		m_event_checks.push_back(cond);

	//		cond = boost::bind(&check_event_level, this, , E_SINE_COMM_INIT_ERR, _1, _2);
	//		m_event_checks.push_back(cond);

	//		cond = boost::bind(&check_event_level, this, ::EMERGENCY_STOP, E_EMERGENCY_STOP, _1, _2);
	//		m_event_checks.push_back(cond);

		//TODO: FIX ME: Second startup creates lots of timeout's.
			cond = boost::bind(&check_event_edge, this, ::TIMEOUT, E_EC_TIMEOUT, m_timeout, _1, _2); //TODO: Set back to level
			m_event_checks.push_back(cond);
	}

	void YouBotBaseService::getControlModes(vector<ctrl_modes>& all)
	{
		all = m_joint_ctrl_modes;
	}

	void YouBotBaseService::setControlModes(vector<ctrl_modes>& all)
	{
		if(all.size() != NR_OF_BASE_SLAVES)
		{
			log(Error) << "The number of ctrl_modes should match the number of motors." << endlog();
			this->getOwner()->error();
			return;
		}

		// If one ctrl_mode is TWIST, check to see if all ctrl_modes are set to TWIST.
		bool twist(false);
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			if(all[i] == TWIST && !twist)
			{
				twist = true;
				if(i != 0)
				{
					this->getOwner()->error();
					log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES << " motors should be set to this!" << endlog();
					return;
				}
			}
			else if(twist && all[i] != TWIST)
			{
				this->getOwner()->error();
				log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES << " motors should be set to this!" << endlog();
				return;
			}
		}

		m_joint_ctrl_modes = all;
	}

	bool YouBotBaseService::start()
	{
		return m_calibrated;
	}

	void YouBotBaseService::setTwistSetpoints()
	{
		cmd_twist.read(m_cmd_twist);

		quantity<si::velocity> longitudinalVelocity = m_cmd_twist.linear.x * si::meter_per_second;
		quantity<si::velocity> transversalVelocity = m_cmd_twist.linear.y * si::meter_per_second;
		quantity<si::angular_velocity> angularVelocity = m_cmd_twist.angular.z * si::radian_per_second;

		m_base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

//		std::vector<quantity<angular_velocity> > wheelVelocities(NR_OF_BASE_SLAVES, 0);
//
//		m_kinematics.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, wheelVelocities);
//
//		for(unsigned int joint_nr = 0; joint_nr < NR_OF_BASE_SLAVES; ++joint_nr)
//		{
//			m_tmp_joint_cmd_velocity.angularVelocity = wheelVelocities[joint_nr];
//			m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
//		}
	}

	void YouBotBaseService::setJointSetpoints()
	{
		joint_cmd_angles.read(m_joint_cmd_angles);
		joint_cmd_velocities.read(m_joint_cmd_velocities);
		joint_cmd_torques.read(m_joint_cmd_torques);

		for(unsigned int joint_nr = 0; joint_nr < NR_OF_BASE_SLAVES; ++joint_nr)
		{
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
				case(TWIST):
				{
					log(Error) << "Cannot be in TWIST ctrl_mode (programming error)" << endlog();
					this->getOwner()->error();
					break;
				}
				default:
				{
					log(Error) << "Case not recognized." << endlog();
					this->getOwner()->error();
					break;
				}
			}
		}
	}

	void YouBotBaseService::readJointStates()
	{
		// YouBot -> OutputPort
		m_base->getJointData(m_tmp_joint_angles);
		m_base->getJointData(m_tmp_joint_velocities);
		m_base->getJointData(m_tmp_joint_torques);
//
		assert(m_tmp_joint_angles.size() == m_tmp_joint_velocities.size() && m_tmp_joint_velocities.size() == m_tmp_joint_torques.size());

		int size = m_tmp_joint_angles.size();
		for(int i = 0; i < size; ++i)
		{
			m_joint_states.position[i] = m_tmp_joint_angles[i].angle.value();

			m_joint_states.velocity[i] = m_tmp_joint_velocities[i].angularVelocity.value();

			m_joint_states.effort[i] = sign(m_joint_states.velocity[i]) * (m_tmp_joint_torques[i].torque.value() - m_torque_offset[i]);
		}
	}

	void YouBotBaseService::calculateOdometry()
	{
		std::vector<quantity<angular_velocity> > wheelVelocities(NR_OF_BASE_SLAVES, 0);
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			wheelVelocities[i] = m_tmp_joint_velocities[i].angularVelocity;
		}

		quantity<si::velocity> longitudinalVelocity;
		quantity<si::velocity> transversalVelocity;
		quantity<angular_velocity> angularVelocity;

		m_base->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

//		m_kinematics.wheelVelocitiesToCartesianVelocity(wheelVelocities, longitudinalVelocity, transversalVelocity, angularVelocity);

		m_odometry_state.twist.twist.linear.x = longitudinalVelocity.value();
		m_odometry_state.twist.twist.linear.y = transversalVelocity.value();
//		m_odometry_state.twist.twist.linear.z = 0;
//		m_odometry_state.twist.twist.angular.x = 0;
//		m_odometry_state.twist.twist.angular.y = 0;
		m_odometry_state.twist.twist.angular.z = angularVelocity.value();

//		std::vector<quantity<plane_angle> > wheelPositions;
//		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
//		{
//			wheelPositions[i] = m_tmp_joint_angles[i].angle;
//		}

		quantity<si::length> longitudinalPosition;
		quantity<si::length> transversalPosition;
		quantity<plane_angle> orientation; //yaw

//		m_kinematics.wheelPositionsToCartesianPosition(wheelPositions, longitudinalPosition, transversalPosition, orientation);

		m_base->getBasePosition(longitudinalPosition, transversalPosition, orientation);

		m_odometry_state.pose.pose.position.x = longitudinalPosition.value();
		m_odometry_state.pose.pose.position.y = transversalPosition.value();
//		m_odometry_state.pose.pose.position.z = 0;

		m_odometry_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation.value());
	}

	void YouBotBaseService::update()
	{
		// Sensors
		ros::Time stamp = ros::Time::now();
		m_joint_states.header.stamp = stamp;
		m_odometry_state.header.stamp = stamp;

		readJointStates();
		calculateOdometry();

		joint_states.write(m_joint_states);
		odometry_state.write(m_odometry_state);

		// Actuators
		if(m_joint_ctrl_modes[0] == TWIST) // All joints will be in TWIST ctrl_mode (see setControlModes)
		{
			setTwistSetpoints();
		}
		else
		{
			setJointSetpoints();
		}

		// Check for errors -> events
		checkMotorStatuses();
	}

	void YouBotBaseService::checkMotorStatuses()
	{
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i]->getStatus(m_motor_statuses.flags[i]);

			for(unsigned int j = 0; j < m_event_checks.size(); ++j)
			{
				m_event_checks[j](i, m_motor_statuses.flags[i]);
			}
		}

		motor_statuses.write(m_motor_statuses);
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
			this->getOwner()->error();
			return false;
		}

		log(Info) << "Calibrated." << endlog();
		return (m_calibrated = true);
	}

	void YouBotBaseService::calibrateTorqueOffset()
	{
		// Workaround for missing current sign + non-linearity.
		JointTorqueSetpoint setp;
		setp.torque = 0.0 * si::newton_meter;
		JointSensedTorque jst;

		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i]->setData(setp);

			m_joints[i]->getData(jst);
			if(m_torque_offset[i] == 0.0)
			{
				m_torque_offset[i] = jst.torque.value();
			}
			else
			{
				m_torque_offset[i] = (m_torque_offset[i] + jst.torque.value()) / 2;
			}
			log(Info) << "Torque offset is: " << m_torque_offset[i] << endlog();
		}
		// end workaround;
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

	void YouBotBaseService::clearControllerTimeouts()
	{
		for(unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
		{
			m_joints[i]->getStatus(m_motor_statuses.flags[i]);
			if( m_motor_statuses.flags[i] & ::TIMEOUT )
			{
				ClearMotorControllerTimeoutFlag clearTimeoutFlag;
				m_joints[i]->setConfigurationParameter(clearTimeoutFlag);
			}
		}
	}

}
