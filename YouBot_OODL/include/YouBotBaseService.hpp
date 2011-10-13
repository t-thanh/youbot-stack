#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotJoint.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;

    class YouBotBaseService : public Service {

		public:
			YouBotBaseService(const string& name, TaskContext* parent, unsigned int min_slave_nr);
			virtual ~YouBotBaseService();

			void getBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation);
			void setBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation);

			void getBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity);
			void setBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity);

			void setControlModes(vector<ctrl_modes>& all);
			vector<ctrl_modes> getControlModes();
			void displayMotorStatuses();

		private:
			OutputPort<sensor_msgs::JointState> joint_states;

			OutputPort<YouBot_OODL::motor_statuses > motor_statuses;

			InputPort<motion_control_msgs::JointVelocities> joint_cmd_velocities;
			InputPort<motion_control_msgs::JointPositions> joint_cmd_angles;
			InputPort<motion_control_msgs::JointEfforts> joint_cmd_torques;

			InputPort<vector<ctrl_modes> > joint_ctrl_modes;

			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			void updateJointSetpoint(unsigned int joint_nr);

			bool check_error();

	        motion_control_msgs::JointVelocities m_joint_cmd_velocities;
	        motion_control_msgs::JointPositions  m_joint_cmd_angles;
	        motion_control_msgs::JointEfforts  m_joint_cmd_torques;
	        sensor_msgs::JointState m_joint_states;

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

			bool m_calibrated;

			const unsigned int m_min_slave_nr;
    };

}
