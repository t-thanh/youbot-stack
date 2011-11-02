#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <motion_control_msgs/JointPositions.h>

namespace YouBot
{
	using namespace RTT;
	typedef std_msgs::Float64MultiArray flat_matrix_t;

	enum state_t {FULL_CONTROL, JOINT_CONTROL, CARTESIAN_CONTROL, GRAVITY_MODE, GUARDED_MOVE, RETRACT_GRIPPER};

	static const unsigned int SIZE_JOINTS_ARRAY = 8;//5 arm + 3  virtual base
	static const unsigned int SIZE_CART_SPACE=6;//
	static const unsigned int SIZE_CART_STIFFNESS=2;
	static const double GRIPPER_OPENING=0.022;

	static const double UNFOLD_JOINT_POSE[]={0,0,0,0,0,0,0,0};
	static const double UNFOLD_CART_POSE[]={0,0,1,0,0,0};
	static const double BASIC_JOINT_STIFFNESS[]={0,0,0,70,50,50,50,50};
	static const double BASIC_CART_STIFFNESS[]={3,70};

	class YouBot_executive: public TaskContext
	{
	public:
		YouBot_executive(const string& name);
		virtual ~YouBot_executive();

		virtual void updateHook();

		void stateTransition(state_t new_state);
		void stateFullControl();
		void stateGravityMode();
		void stateJointControl();
		void stateCartesianControl();
		void stateGuardedMove();

		void doneEvent();

		// Operations
		void unfoldArm(); 	// operation
		void gravityMode();			// operation
		void positionArm(vector<double> position_j);// operation
		void positionGripper(vector<double> position_c);// operation
		void getArmPose(vector<double>& position_c); // operation
		void getGripperPose(vector<double>& position_j); // operation
		void getGripperH(vector<double>& H);
		void setCartesianStiffness(vector<double> stiffness_c);// operation
		void setJointStiffness(vector<double> stiffness_j);// operation
		void guardMove(vector<double> force_c);// operation
		void openGripper();// operation
		void closeGripper();// operation

		// Ports and their variables
		RTT::OutputPort<flat_matrix_t> JointSpaceSetpoint;
		RTT::OutputPort<flat_matrix_t> JointSpaceStiffness;
		RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
		RTT::OutputPort<flat_matrix_t> CartSpaceStiffness_rot;
		RTT::OutputPort<flat_matrix_t> CartSpaceStiffness_trans;

		RTT::InputPort<flat_matrix_t> CartGripperPose; // Homogeneous matrix
		RTT::InputPort<flat_matrix_t> JointGripperPose;
		RTT::InputPort<flat_matrix_t> CartForceState;

		RTT::OutputPort<motion_control_msgs::JointPositions> gripper_cmd;

		RTT::OutputPort<std::string> events;

	protected:
		void setupComponentInterface();
		void init();

		// The following are set from operations.
		vector<double> m_position_jnt;
		vector<double> m_stiffness_jnt;
		vector<double> m_position_cart;
		vector<double> m_stiffness_cart;
		vector<double> m_force_cart;

		// Variables for ports
		flat_matrix_t m_JointSpaceSetpoint;
		flat_matrix_t m_JointSpaceStiffness;
		flat_matrix_t m_CartSpaceSetpoint;
		flat_matrix_t m_CartSpaceStiffness_rot;
		flat_matrix_t m_CartSpaceStiffness_trans;

		flat_matrix_t m_CartGripperPose;
		flat_matrix_t m_JointGripperPose;
		flat_matrix_t m_CartForceState;

		motion_control_msgs::JointPositions m_gripper_cmd;

		state_t m_state;

		std::string m_events;
	};
}

