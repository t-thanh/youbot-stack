#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotJoint.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#include "YouBotService.hpp"
#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;

    class YouBotBaseService : public YouBotService {

		public:
			YouBotBaseService(const string& name, TaskContext* parent, unsigned int min_slave_nr);
			virtual ~YouBotBaseService();

			void setControlModes(vector<ctrl_modes>& all);
			void getControlModes(vector<ctrl_modes>& all);

			void displayMotorStatuses();

			void clearControllerTimeouts();
		private:
			OutputPort<sensor_msgs::JointState> joint_states;
			OutputPort<nav_msgs::Odometry> odometry_state;

			OutputPort<YouBot_OODL::motor_statuses > motor_statuses;

			InputPort<motion_control_msgs::JointVelocities> joint_cmd_velocities;
			InputPort<motion_control_msgs::JointPositions> joint_cmd_angles;
			InputPort<motion_control_msgs::JointEfforts> joint_cmd_torques;

			InputPort<geometry_msgs::Twist> cmd_twist;

			void setupComponentInterface();
			void setupEventChecks();

			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			void readJointStates();
			void calculateOdometry();

			void setJointSetpoints();
			void setTwistSetpoints();

			void checkMotorStatuses();

	        motion_control_msgs::JointVelocities m_joint_cmd_velocities;
	        motion_control_msgs::JointPositions  m_joint_cmd_angles;
	        motion_control_msgs::JointEfforts  m_joint_cmd_torques;
	        geometry_msgs::Twist m_cmd_twist;

	        sensor_msgs::JointState m_joint_states;
	        nav_msgs::Odometry m_odometry_state;

			vector<JointSensedAngle> m_tmp_joint_angles;
			vector<JointSensedVelocity> m_tmp_joint_velocities;
			vector<JointSensedTorque> m_tmp_joint_torques;

			YouBot_OODL::motor_statuses m_motor_statuses;
			vector<ctrl_modes> m_joint_ctrl_modes;

			JointAngleSetpoint m_tmp_joint_cmd_angle;
			JointVelocitySetpoint m_tmp_joint_cmd_velocity;
			JointTorqueSetpoint m_tmp_joint_cmd_torque;

			YouBotBase* m_base;
			YouBotJoint* m_joints[NR_OF_BASE_SLAVES];

			std::vector<check_fp> m_event_checks;
			bool m_overcurrent[NR_OF_ARM_SLAVES];
			bool m_undervoltage[NR_OF_ARM_SLAVES];
			bool m_overvoltage[NR_OF_ARM_SLAVES];
			bool m_overtemperature[NR_OF_ARM_SLAVES];
			bool m_connectionlost[NR_OF_ARM_SLAVES];
			bool m_i2texceeded[NR_OF_ARM_SLAVES];

			bool m_calibrated;

			const unsigned int m_min_slave_nr;
    };

}
