#include "YouBot_executive.h"
#include <tf/transform_broadcaster.h>
#include "ExecutiveHelpers.hpp"

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

YouBot_executive::YouBot_executive(const string& name) :
	TaskContext(name)
{
	this->init();

	this->setupComponentInterface();
}

YouBot_executive::~YouBot_executive()
{ }

void YouBot_executive::setupComponentInterface()
{
	this->addOperation("unfoldArm", &YouBot_executive::unfoldArm, this).doc("Unfold the arm");

	this->addOperation("gravityMode", &YouBot_executive::gravityMode, this).doc("Set gravity compensation mode");

	this->addOperation("positionArm", &YouBot_executive::positionArm, this).doc("Joint space control");

	this->addOperation("positionGripper", &YouBot_executive::positionGripper, this).doc("Cartesian space control");

	this->addOperation("getArmPose", &YouBot_executive::getArmPose, this).doc("Get joint space positions");

	this->addOperation("getGripperPose", &YouBot_executive::getGripperPose,this).doc("Get cartesian space positions");

	this->addOperation("setCartesianStiffness", &YouBot_executive::setCartesianStiffness,this).doc(" ");
	this->addOperation("setJointStiffness", &YouBot_executive::setJointStiffness,this).doc(" ");

	this->addOperation("openGripper", &YouBot_executive::openGripper, this).doc(" ");
	this->addOperation("closeGripper", &YouBot_executive::closeGripper, this).doc(" ");
	this->addOperation("retractGripper",&YouBot_executive::retractGripper,this).doc("");

	this->addOperation("guardMove", &YouBot_executive::guardMove, this).doc(
			"Performs guarded move based on the set force, issue event e_done. NOTE: the edge of working envelope considered as obstacle. If the force 	limit is higher then max force allowed in controller e_done event will be never sent.");

	this->addPort("JointSpaceSetpoint", JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffness", JointSpaceStiffness).doc("");
	this->addPort("CartSpaceSetpoint", CartSpaceSetpoint).doc("");
	this->addPort("CartSpaceStiffness_rot", CartSpaceStiffness_rot).doc("");
	this->addPort("CartSpaceStiffness_trans", CartSpaceStiffness_trans).doc("");

	this->addPort("GripperPose", CartGripperPose).doc("");
	this->addPort("ArmPose", JointGripperPose).doc("");
	this->addPort("gripper_cmd", gripper_cmd).doc("");
	this->addPort("events_out", events).doc("");
	this->addPort("CartForceState", CartForceState).doc("Input from the control");

	// Debugging/introspection properties
	this->addProperty("Joint_position_setpoint", m_position_jnt);
	this->addProperty("Joint_stiffness_setpoint", m_stiffness_jnt);
	this->addProperty("Gripper_position_setpoint", m_position_cart);
	this->addProperty("Gripper_stiffness_setpoint", m_stiffness_cart);
	this->addProperty("Force_guard",m_force_cart);
}

void YouBot_executive::init()
{
	m_position_jnt.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_stiffness_jnt.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_position_cart.resize(SIZE_CART_SPACE, 0.0);
	m_stiffness_cart.resize(SIZE_CART_STIFFNESS, 0.0);
	m_force_cart.resize(SIZE_CART_SPACE, 0.0);

	// Variables for ports
	m_JointSpaceSetpoint.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_JointSpaceStiffness.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_CartSpaceSetpoint.data.resize(SIZE_CART_SPACE, 0.0);
	m_CartSpaceStiffness_rot.data.resize(1, 0.0);
	m_CartSpaceStiffness_trans.data.resize(1, 0.0);

	m_CartGripperPose.data.resize(16, 0.0);
	m_JointGripperPose.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_CartForceState.data.resize(SIZE_CART_SPACE, 0.0);

	m_gripper_cmd.positions.resize(1, 0.0);

	m_state = GRAVITY_MODE;

	m_events.reserve(50);

	JointSpaceSetpoint.setDataSample(m_JointSpaceSetpoint);
	JointSpaceStiffness.setDataSample(m_JointSpaceStiffness);
	CartSpaceSetpoint.setDataSample(m_CartSpaceSetpoint);
	CartSpaceStiffness_rot.setDataSample(m_CartSpaceStiffness_rot);
	CartSpaceStiffness_trans.setDataSample(m_CartSpaceStiffness_trans);
	gripper_cmd.setDataSample(m_gripper_cmd);
}

void YouBot_executive::openGripper()
{
	m_gripper_cmd.positions[0] = GRIPPER_OPENING;
	gripper_cmd.write(m_gripper_cmd);
}

void YouBot_executive::closeGripper()
{
	m_gripper_cmd.positions[0] = 0.0001;
	gripper_cmd.write(m_gripper_cmd);
}

void YouBot_executive::unfoldArm()
{
	//RTT::log(Info) << "Call unfoldArm" << endlog();
	m_position_cart.assign(UNFOLD_CART_POSE,UNFOLD_CART_POSE+6);
	m_position_jnt.assign(UNFOLD_JOINT_POSE,UNFOLD_JOINT_POSE+8);
	m_stiffness_cart.assign(BASIC_CART_STIFFNESS,BASIC_CART_STIFFNESS+2);
	m_stiffness_jnt.assign(BASIC_JOINT_STIFFNESS,BASIC_JOINT_STIFFNESS+8);

	stateTransition(FULL_CONTROL);
}

void YouBot_executive::gravityMode()
{
	stateTransition(GRAVITY_MODE);
}

void YouBot_executive::setCartesianStiffness(vector<double> stiffness_c)
{
	if(stiffness_c.size() != SIZE_CART_STIFFNESS)
	{
		log(Error) << "setCartesianStiffness - expects a " << SIZE_CART_STIFFNESS << " dimensional vector" << endlog();
		return;
	}
	m_stiffness_cart.assign(stiffness_c.begin(),stiffness_c.end());
}

void YouBot_executive::setJointStiffness(vector<double> stiffness_j)
{
	if(stiffness_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "setJointStiffness - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}
	m_stiffness_jnt.assign(stiffness_j.begin(),stiffness_j.end()); //Swap is valid since
}

void YouBot_executive::positionArm(vector<double> position_j)
{
	if(position_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "positionArm - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}

	m_position_jnt.assign(position_j.begin(),position_j.end());
	stateTransition(JOINT_CONTROL);
}

void YouBot_executive::positionGripper(vector<double> position_c)
{
	if(position_c.size() != SIZE_CART_SPACE)
	{
		log(Error) << "positionGripper - expects a " << SIZE_CART_SPACE << " dimensional vector" << endlog();
		return;
	}

	m_position_cart.assign(position_c.begin(),position_c.end());
	stateTransition(CARTESIAN_CONTROL);
}

void YouBot_executive::getArmPose(vector<double> & position_j)
{
	if(position_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "getArmPose - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}

	position_j.assign(m_JointGripperPose.data.begin(), m_JointGripperPose.data.end());
}

void YouBot_executive::getGripperPose(vector<double> & position_c)
{
	if(position_c.size() != SIZE_CART_SPACE)
	{
		log(Error) << "getGripperPose - expects a " << SIZE_CART_SPACE << " dimensional vector" << endlog();
		return;
	}

	homogeneous_to_xyzypr(m_CartGripperPose.data, position_c);
}

void YouBot_executive::retractGripper()
{
	stateTransition(RETRACT_GRIPPER);
}

void YouBot_executive::getGripperH(vector<double>& H)
{
	if(H.size() != 16)
	{
		log(Error) << "getGripperH - expects a 16 dimensional vector" << endlog();
		return;
	}

	H.assign(m_CartGripperPose.data.begin(), m_CartGripperPose.data.end());
}

void YouBot_executive::guardMove(vector<double> force_c)
{
	if(force_c.size() != 3)
	{
		log(Error) << "guardMove - expects a 3 dimensional vector" << endlog();
		return;
	}

	m_force_cart = force_c;
	stateTransition(GUARDED_MOVE);
}

void YouBot_executive::doneEvent(){
	m_events = "executive.e_done";
	events.write(m_events);
}

const double RETRACT_STIFFNESS_C[2]={50,500};
const double GRIPPER_SIZE[3]={0,0,-0.2,1};

void YouBot_executive::stateTransition(state_t new_state)
{
	// Includes init functions
	if(new_state == RETRACT_GRIPPER)
	{
		vector<double> vecGripperSize(4, 0.0);
		vecGripperSize.assign(GRIPPER_SIZE, GRIPPER_SIZE+4);
		vector<double> error(4, 0.0); //only translation
		vector<double> setPoint(SIZE_CART_SPACE, 0.0);
		vector<double> states_c(SIZE_CART_SPACE , 0.0);

		homogeneous_to_xyzypr(m_CartGripperPose.data, states_c);

		// Calculate new virtual cartesian setpoint and stiffness once
		Multiply(m_CartGripperPose.data, vecGripperSize, error); //H, 3, output
		setPoint[0]=error[0];
		setPoint[1]=error[1];
		setPoint[2]=error[2];
		setPoint[3]=states_c[3];
		setPoint[4]=states_c[4];
		setPoint[5]=states_c[5];

		m_position_cart.assign(setPoint.begin(), setPoint.end());
		m_stiffness_cart.assign(RETRACT_STIFFNESS_C, RETRACT_STIFFNESS_C+2);

		// Afterwards, just use cartesian control
		m_state = CARTESIAN_CONTROL;
	}
	else
	{
		m_state = new_state;
	}
}

void YouBot_executive::updateHook()
{
	// read all input ports
	CartGripperPose.read(m_CartGripperPose);
	JointGripperPose.read(m_JointGripperPose);
	CartForceState.read(m_CartForceState);

	// perform the state specific actions
	switch(m_state)
	{
		case(FULL_CONTROL):
		{
			stateFullControl();
			break;
		}
		case(GRAVITY_MODE):
		{
			stateGravityMode();
			break;
		}
		case(JOINT_CONTROL):
		{
			stateJointControl();
			break;
		}
		case(CARTESIAN_CONTROL):
		{
			stateCartesianControl();
			break;
		}
		case(RETRACT_GRIPPER):
		{
			log(Error) << "RETRACT_GRIPPER is NOT a real state, but implemented by CARTESIAN_CONTROL" << endlog();
			this->error();
			break;
		}
		case(GUARDED_MOVE):
		{
			stateGuardedMove();
			break;
		}
		default:
		{
			log(Error) << "control_state not recognized." << endlog();
			this->error();
			return;
			break;
		}
	}

	// write setpoints
	JointSpaceSetpoint.write(m_JointSpaceSetpoint);
	JointSpaceStiffness.write(m_JointSpaceStiffness);
	CartSpaceSetpoint.write(m_CartSpaceSetpoint);
	CartSpaceStiffness_rot.write(m_CartSpaceStiffness_rot);
	CartSpaceStiffness_trans.write(m_CartSpaceStiffness_trans);
}

void YouBot_executive::stateFullControl()
{
	// Copies the given setpoints plain to the outputs.

	// Assign the setpoints
	m_JointSpaceSetpoint.data.assign(m_position_jnt.begin(), m_position_jnt.end());
	m_CartSpaceSetpoint.data.assign(m_position_cart.begin(), m_position_cart.end());

	m_JointSpaceStiffness.data.assign(m_stiffness_jnt.begin(), m_stiffness_jnt.end());
	m_CartSpaceStiffness_rot.data.at(0)=m_stiffness_cart.at(0);
	m_CartSpaceStiffness_trans.data.at(0)=m_stiffness_cart.at(1);
}

void YouBot_executive::stateGravityMode()
{
	// Assigns the current states as setpoints and sets the stiffness zero.

	homogeneous_to_xyzypr(m_CartGripperPose.data, m_CartSpaceSetpoint.data);
	m_JointSpaceSetpoint.data.assign(m_JointGripperPose.data.begin(), m_JointGripperPose.data.end());

	double stiffness_jnt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	m_JointSpaceStiffness.data.assign(stiffness_jnt, stiffness_jnt + 8);
	m_CartSpaceStiffness_rot.data.at(0)= 0;
	m_CartSpaceStiffness_trans.data.at(0)= 0;
}

void YouBot_executive::stateJointControl()
{
	// Copy the joint space setpoints and cartesian states plain to the outputs.

	// From inputs
	m_JointSpaceSetpoint.data.assign(m_position_jnt.begin(), m_position_jnt.end());

	// From states
	homogeneous_to_xyzypr(m_CartGripperPose.data, m_CartSpaceSetpoint.data);

	// From input
	m_JointSpaceStiffness.data.assign(m_stiffness_jnt.begin(), m_stiffness_jnt.end());

	// Cartesian stiffness zero
	m_CartSpaceStiffness_rot.data.at(0)=0;
	m_CartSpaceStiffness_trans.data.at(0)=0;
}

void YouBot_executive::stateCartesianControl()
{
	// Copy the joint states and cartesian setpoints plain to the outputs.

	// From states
	m_JointSpaceSetpoint.data.assign(m_JointGripperPose.data.begin(), m_JointGripperPose.data.end());

	// From inputs
	m_CartSpaceSetpoint.data.assign(m_position_cart.begin(), m_position_cart.end());

	// Joint space zero stiffness
	double stiffness_jnt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	m_JointSpaceStiffness.data.assign(stiffness_jnt, stiffness_jnt + 8);

	// From input
	m_CartSpaceStiffness_rot.data.at(0)=m_stiffness_cart.at(0);
	m_CartSpaceStiffness_trans.data.at(0)=m_stiffness_cart.at(1);
}

const double GUARD_STIFFNESS_C[]={2,70};

void YouBot_executive::stateGuardedMove()
{
	// Move with a predefined force in cartesian space
	vector<double> error(4, 0.0);
	bool done = true;

	vector<double> temp(4, 0.0);
	temp[0]=m_force_cart[0];
	temp[1]=m_force_cart[1];
	temp[2]=m_force_cart[2];
	temp[3]=1;

	// Calculate forces
	Multiply(m_CartGripperPose.data,temp,error);

	for(unsigned int i=0; i<3;i++)
	{
		done = done && ( abs(temp[i] + m_CartForceState.data[i+3]) < alpha ); // m_CartForceState is in w,x order
		m_position_cart[i] = error[i];
	}

	// From states
	m_JointSpaceSetpoint.data.assign(m_JointGripperPose.data.begin(), m_JointGripperPose.data.end());

	// Calculated inputs
	m_CartSpaceSetpoint.data.assign(m_position_cart.begin(), m_position_cart.end());

	// Joint space zero stiffness
	double stiffness_jnt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	m_JointSpaceStiffness.data.assign(stiffness_jnt, stiffness_jnt + 8);

	// Use pre set stiffnesses (for now) // From input
	m_CartSpaceStiffness_rot.data[0] = GUARD_STIFFNESS_C[0];//m_stiffness_cart[0];
	m_CartSpaceStiffness_trans.data[0] = GUARD_STIFFNESS_C[1];//m_stiffness_cart[1];
}

}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)
