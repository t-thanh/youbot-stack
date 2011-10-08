#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>
#include <youbot/YouBotGripper.hpp>

#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;
	using namespace boost::units;
	using namespace boost::units::si;

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
	typedef struct _JointLimits JointLimits;

	/**
	 * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
	 */
	struct _GripperLimits
	{
		quantity<si::length> min_position;
		quantity<si::length> max_position;

		_GripperLimits() : min_position(0*meter), max_position(0*meter)
		{}
	};
	typedef struct _GripperLimits GripperLimits;

    class YouBotManipulatorService : public Service {

		public:
    		YouBotManipulatorService(const string& name, TaskContext* parent, unsigned int min_slave_nr);
			virtual ~YouBotManipulatorService();

			void setControlModes(vector<ctrl_modes>& all);
			void getControlModes(vector<ctrl_modes>& all);

			void displayJointStatuses();

			static unsigned int non_errors;

		protected:
			// Joints
			OutputPort<sensor_msgs::JointState> joint_states;

			OutputPort<vector<joint_status> > joint_statuses;

			InputPort<motion_control_msgs::JointPositions> joint_cmd_angles;
			InputPort<motion_control_msgs::JointVelocities> joint_cmd_velocities;
			InputPort<motion_control_msgs::JointEfforts> joint_cmd_torques;

			InputPort<vector<ctrl_modes> > joint_ctrl_modes;

			// Gripper
			InputPort<motion_control_msgs::JointPositions> gripper_cmd_position;
			OutputPort<sensor_msgs::JointState> gripper_state;

		private:
			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			void updateJointSetpoint(unsigned int joint_nr);

			bool check_error();

			// Joints
	        motion_control_msgs::JointVelocities m_joint_cmd_velocities;
	        motion_control_msgs::JointPositions  m_joint_cmd_angles;
	        motion_control_msgs::JointEfforts  m_joint_cmd_torques;
	        sensor_msgs::JointState m_joint_states;

			vector<JointSensedAngle> m_tmp_joint_angles;
			vector<JointSensedVelocity> m_tmp_joint_velocities;
			vector<JointSensedTorque> m_tmp_joint_torques;

			vector<JointLimits> m_joint_limits;

			vector<joint_status> m_joint_statuses;
			vector<ctrl_modes> m_joint_ctrl_modes;

			JointAngleSetpoint m_tmp_joint_cmd_angle;
			JointVelocitySetpoint m_tmp_joint_cmd_velocity;
			JointTorqueSetpoint m_tmp_joint_cmd_torque;

			// Gripper
	        motion_control_msgs::JointPositions m_gripper_cmd_position;
//	        sensor_msgs::JointState m_gripper_state;
	        GripperBarSpacingSetPoint m_tmp_gripper_cmd_position;
//	        GripperBarSpacingSetPoint m_tmp_gripper_state;
	        GripperLimits m_gripper_limits;

			YouBotManipulator* m_manipulator;
			YouBotJoint* m_joints[NR_OF_ARM_SLAVES];
			YouBotGripper* m_gripper;

			bool m_calibrated;

			const unsigned int m_min_slave_nr;
    };

}
