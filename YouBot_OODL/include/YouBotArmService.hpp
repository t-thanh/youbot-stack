#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>

#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"
#include "YouBotService.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;
	using namespace boost::units;
	using namespace boost::units::si;

	struct _JointLimits;
	typedef struct _JointLimits JointLimits;

    class YouBotArmService : public YouBotService {

		public:
    		YouBotArmService(const string& name, TaskContext* parent, unsigned int min_slave_nr);
			virtual ~YouBotArmService();

			void setControlModes(vector<ctrl_modes>& all);
			void getControlModes(vector<ctrl_modes>& all);

			void displayMotorStatuses();

			void clearControllerTimeouts();

			void calibrateTorqueOffset();

		protected:
			OutputPort<sensor_msgs::JointState> joint_states;

			InputPort<motion_control_msgs::JointPositions> joint_cmd_angles;
			InputPort<motion_control_msgs::JointVelocities> joint_cmd_velocities;
			InputPort<motion_control_msgs::JointEfforts> joint_cmd_torques;

			OutputPort<YouBot_OODL::motor_statuses> motor_statuses;

		private:
			void setupComponentInterface();
			void setupEventChecks();

			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			void readJointStates();
			void updateJointSetpoints();
			void checkMotorStatuses();

	        motion_control_msgs::JointVelocities m_joint_cmd_velocities;
	        motion_control_msgs::JointPositions  m_joint_cmd_angles;
	        motion_control_msgs::JointEfforts  m_joint_cmd_torques;

	        sensor_msgs::JointState m_joint_states;

			vector<JointSensedAngle> m_tmp_joint_angles;
			vector<JointSensedVelocity> m_tmp_joint_velocities;
			vector<JointSensedTorque> m_tmp_joint_torques;

			vector<JointLimits> m_joint_limits;

			vector<ctrl_modes> m_joint_ctrl_modes;

			JointAngleSetpoint m_tmp_joint_cmd_angle;
			JointVelocitySetpoint m_tmp_joint_cmd_velocity;
			JointTorqueSetpoint m_tmp_joint_cmd_torque;

			YouBot_OODL::motor_statuses m_motor_statuses;

			std::vector<check_fp> m_event_checks;
			bool m_overcurrent[NR_OF_ARM_SLAVES];
			bool m_undervoltage[NR_OF_ARM_SLAVES];
			bool m_overvoltage[NR_OF_ARM_SLAVES];
			bool m_overtemperature[NR_OF_ARM_SLAVES];
			bool m_connectionlost[NR_OF_ARM_SLAVES];
			bool m_i2texceeded[NR_OF_ARM_SLAVES];
			bool m_timeout[NR_OF_ARM_SLAVES];

			YouBotManipulator* m_manipulator;
			YouBotJoint* m_joints[NR_OF_ARM_SLAVES];

			// Workaround for missing current sign and the non-linearity (when only the sign is flipped on negative velocities).
			std::vector<double> m_torque_offset;

			bool m_calibrated;

			const unsigned int m_min_slave_nr;
    };

	/**
	 * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
	 */
    struct _JointLimits
	{
		quantity<plane_angle> min_angle;
		quantity<plane_angle> max_angle;
//		double min_velocity;
//		double max_velocity;
//		double min_torque;
//		double max_torque;

		_JointLimits() : min_angle(0*radian), max_angle(0*radian) //, min_velocity(0), max_velocity(0), min_torque(0), max_torque(0)
		{}
	};

}
