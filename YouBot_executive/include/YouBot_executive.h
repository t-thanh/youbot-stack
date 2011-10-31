#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <motion_control_msgs/JointPositions.h>
#include "Macho.hpp"
#include "FlowControl.hpp"

namespace YouBot
{

using namespace RTT;
typedef std_msgs::Float64MultiArray flat_matrix_t;
class YouBot_executive: public TaskContext
{
private:
	static const int SIZE_JOINTS_ARRAY = 8;//5 arm + 3  virtual base
	static const int SIZE_CART_SPACE=6;//
	static const int SIZE_CART_STIFFNESS=2;
	static const double GRIPPER_OPENING=0.022;
	std_msgs::Float64MultiArray msgs_setpoint_j;
	std_msgs::Float64MultiArray msgs_setpoint_c;
	std_msgs::Float64MultiArray msgs_stiffness_j;
	std_msgs::Float64MultiArray msgs_stiffness_c_r;
	std_msgs::Float64MultiArray msgs_stiffness_c_t;
	std_msgs::Float64MultiArray msgs_force_c;
	motion_control_msgs::JointPositions msgs_gripper_cmd;
public:
	static const double UNFOLD_JOINT_POSE[SIZE_JOINTS_ARRAY];
	static const double UNFOLD_CART_POSE[SIZE_CART_SPACE];
	static const double BASIC_JOINT_STIFFNESS[SIZE_JOINTS_ARRAY];
	static const double BASIC_CART_STIFFNESS[SIZE_CART_STIFFNESS];
	Macho::Machine<FlowControl::Top> m_FlowControl;
	YouBot_executive(const string& name);
	virtual ~YouBot_executive();



	void getSetPoints(vector<double>& position_j,vector<double>& position_c);
	void getStates(vector<double>& position_j,vector<double>& position_c);
	void getStiffness(vector<double>& stiffness_j,vector<double>& stiffness_c);
	void getZeroStiffness(vector<double>& stiffness_j,vector<double>& stiffness_c);
	void getGuardForce(vector<double>& force_c);
	void getStateForce(vector<double>& force_c);
	void doneEvent();

	void unfoldArm(); 	// operation
	void gravityMode();			// operation
	void positionArm(vector<double> position_j);// operation
	void positionGripper(vector<double> position_c);// operation
	void getArmPose(vector<double>& position_c); // operation
	void getGripperPose(vector<double>& position_j); // operation

	void setCartesianStiffness(vector<double> stiffness_c);// operation
	void setJointStiffness(vector<double> stiffness_j);// operation
	void guardMove(vector<double>  force_c);// operation
	void openGripper();// operation
	void closeGripper();// operation
	void init();// operation


	vector<double> m_position_j;
	vector<double> m_stiffness_j;
	vector<double> m_position_c;
	vector<double> m_stiffness_c;
	vector<double> m_force_c;

	RTT::OutputPort<flat_matrix_t> m_JointSpaceSetpoint;
	RTT::OutputPort<flat_matrix_t> m_JointSpaceStiffness;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceSetpoint;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceStiffness_r;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceStiffness_t;
	RTT::InputPort<flat_matrix_t> m_CartGripperPose;
	RTT::InputPort<flat_matrix_t> m_JointGripperPose;
	RTT::InputPort<flat_matrix_t> m_CartForceState;
	RTT::OutputPort<motion_control_msgs::JointPositions> m_gripper_cmd;
	RTT::OutputPort<std::string> m_events;

	void writeSetpoints(const vector<double>& position_j,const vector<double>& stiffness_j,
			const vector<double>& position_c, const vector<double>& stiffness_c);
protected:
	void getXYZYPR(const vector<double>& H, vector<double>& XYZYPR);
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();


};
}

