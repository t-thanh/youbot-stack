#include "YouBot_executive.h"
#include <tf/transform_broadcaster.h>

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{
 const double YouBot_executive:: UNFOLD_JOINT_POSE[]={0,0,0,0,0,0,0,0};
 const double YouBot_executive::UNFOLD_CART_POSE[]={0,0,1,0,0,0};
 const double YouBot_executive::BASIC_JOINT_STIFFNESS[]={0,0,0,70,50,50,50,50};
 const double YouBot_executive::BASIC_CART_STIFFNESS[]={3,70};
YouBot_executive::YouBot_executive(const string& name) :
		TaskContext(name, PreOperational),
		m_position_j(),
		m_stiffness_j(),
		m_position_c(),
		m_stiffness_c(),
		m_force_c()
{
	m_position_j.assign(SIZE_JOINTS_ARRAY, 0);
	m_stiffness_j.assign(SIZE_JOINTS_ARRAY, 0);
	m_position_c.assign(SIZE_CART_SPACE, 0);
	m_force_c.assign(SIZE_CART_SPACE,0);
	m_stiffness_c.assign(SIZE_CART_STIFFNESS, 0);
	msgs_setpoint_j.data.assign(SIZE_JOINTS_ARRAY, 0);
	msgs_setpoint_c.data.assign(SIZE_CART_SPACE, 0);
	msgs_stiffness_j.data.assign(SIZE_JOINTS_ARRAY, 0);
	msgs_stiffness_c_r.data.assign(1, 0);
	msgs_stiffness_c_t.data.assign(1, 0);
	//Operation<void(void)> op_unfoldArm("unfoldArm",&YouBot_executive::unfoldArm,this);
	this->addOperation("unfoldArm", &YouBot_executive::unfoldArm, this).doc(
			"jut text");

	//Operation<void(void)> op_gravityMode("gravityMode",&YouBot_executive::gravityMode,OwnThread);
	this->addOperation("gravityMode", &YouBot_executive::gravityMode, this).doc(
			"jut text");

	//Operation<void(void)> op_positionArm("positionArm",&YouBot_executive::positionArm,OwnThread);
	this->addOperation("positionArm", &YouBot_executive::positionArm, this).doc(
			"jut text");
	this->addOperation("positionGripper", &YouBot_executive::positionGripper,
			this).doc("jut text");

	this->addOperation("getArmPose", &YouBot_executive::getArmPose, this).doc(
			"jut text");
	this->addOperation("getGripperPose", &YouBot_executive::getGripperPose,
			this).doc("jut text");
	this->addOperation("init",&YouBot_executive::init,this).doc(" set data samples and clean up errors");
	this->addOperation("setCartesianStiffness", &YouBot_executive::setCartesianStiffness,this).doc(" ");
	this->addOperation("setJointStiffness", &YouBot_executive::setJointStiffness,this).doc(" ");

	this->addOperation("openGripper", &YouBot_executive::openGripper, this).doc(" ");
	this->addOperation("closeGripper", &YouBot_executive::closeGripper, this).doc(
			" ");
	this->addOperation("guardMove", &YouBot_executive::guardMove, this).doc(
			"Performs guarded move based on the set force, issue event e_done. NOTE: the edge of working envelope considered as obstacle. If the force 	limit is higher then max force allowed in controller e_done event will be never sent.");

	this->addPort("JointSpaceSetpoint", m_JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffness", m_JointSpaceStiffness).doc("");
	this->addPort("CartSpaceSetpoint", m_CartSpaceSetpoint).doc("");
	this->addPort("CartSpaceStiffness_rot", m_CartSpaceStiffness_r).doc("");
	this->addPort("CartSpaceStiffness_trans", m_CartSpaceStiffness_t).doc("");

	this->addPort("GripperPose", m_CartGripperPose).doc("");
	this->addPort("ArmPose", m_JointGripperPose).doc("");
	this->addPort("gripper_cmd", m_gripper_cmd).doc("");
	this->addPort("events_out", m_events).doc("");
	this->addPort("CartForceState", m_CartForceState).doc("Input from the control");

	this->addProperty("Joint_position_setpoint", m_position_j);
	this->addProperty("Joint_stiffness_setpoint", m_stiffness_j);
	this->addProperty("Gripper_position_setpoint", m_position_c);
	this->addProperty("Gripper_stiffness_setpoint", m_stiffness_c);
	this->addProperty("Force_guard",m_force_c);

	this->init();
}
void YouBot_executive::setCartesianStiffness(vector<double> stiffness_c)
{
	//not thread safe
	m_stiffness_c.assign(stiffness_c.begin(),stiffness_c.end());
}
void YouBot_executive::setJointStiffness(vector<double> stiffness_j)
{
	//not thread safe
	m_stiffness_j.assign(stiffness_j.begin(),stiffness_j.end()); //Swap is valid since
}
void YouBot_executive::init()
{
	msgs_setpoint_j.data.assign(SIZE_JOINTS_ARRAY, 0);
	msgs_setpoint_c.data.assign(SIZE_CART_SPACE, 0);
	msgs_stiffness_j.data.assign(SIZE_JOINTS_ARRAY, 0);
	msgs_stiffness_c_r.data.assign(1, 0);
	msgs_stiffness_c_t.data.assign(1, 0);
	msgs_gripper_cmd.positions.assign(1,0);
	setPeriod(0.01);
	m_JointSpaceSetpoint.setDataSample(msgs_setpoint_j);
	m_JointSpaceStiffness.setDataSample(msgs_stiffness_j);
	m_CartSpaceSetpoint.setDataSample(msgs_setpoint_c);
	m_CartSpaceStiffness_r.setDataSample(msgs_stiffness_c_r);
	m_CartSpaceStiffness_t.setDataSample(msgs_stiffness_c_t);
	m_gripper_cmd.setDataSample(msgs_gripper_cmd);



}
void YouBot_executive::openGripper()
{

	msgs_gripper_cmd.positions.at(0)=GRIPPER_OPENING;
	m_gripper_cmd.write(msgs_gripper_cmd);

}
void YouBot_executive::closeGripper(){
	msgs_gripper_cmd.positions.at(0)=0.0001;
	m_gripper_cmd.write(msgs_gripper_cmd);

}
YouBot_executive::~YouBot_executive()
{

}

bool YouBot_executive::configureHook()
{
	return TaskContext::configureHook();
}

bool YouBot_executive::startHook()
{

	return TaskContext::startHook();
}

void YouBot_executive::updateHook()
{
	m_FlowControl->run(this);
}

void YouBot_executive::stopHook()
{

	TaskContext::stopHook();
}

void YouBot_executive::getSetPoints(vector<double> & position_j,
		vector<double> & position_c)
{
	position_j.assign(m_position_j.begin(), m_position_j.end());
	position_c.assign(m_position_c.begin(), m_position_c.end());
}
void YouBot_executive::getStates(vector<double> & position_j,
		vector<double> & position_c)
{
	getArmPose(position_j);
	getGripperPose(position_c);
}
void YouBot_executive::getStiffness(vector<double> & stiffness_j,
		vector<double> & stiffness_c)
{
	stiffness_j.assign(m_stiffness_j.begin(), m_stiffness_j.end());
	stiffness_c.assign(m_stiffness_c.begin(), m_stiffness_c.end());
}

void YouBot_executive::getZeroStiffness(vector<double> & stiffness_j,
		vector<double> & stiffness_c)
{
	double d_stiffness_j[8] =
	{ 0, 0, 0, 0, 0, 0, 0, 0 };
	double d_stiffness_c[2] =
	{ 0, 0 };
	stiffness_j.assign(d_stiffness_j, d_stiffness_j + 8);
	stiffness_c.assign(d_stiffness_c, d_stiffness_c + 2);
}

void YouBot_executive::cleanupHook()
{
	TaskContext::cleanupHook();
}

void YouBot_executive::unfoldArm()
{
	//RTT::log(Info) << "Call unfoldArm" << endlog();
	m_position_c.assign(UNFOLD_CART_POSE,UNFOLD_CART_POSE+6);
	m_position_j.assign(UNFOLD_JOINT_POSE,UNFOLD_JOINT_POSE+8);
	m_stiffness_c.assign(BASIC_CART_STIFFNESS,BASIC_CART_STIFFNESS+2);
	m_stiffness_j.assign(BASIC_JOINT_STIFFNESS,BASIC_JOINT_STIFFNESS+8);
	m_FlowControl->e_fullControl();
}

void YouBot_executive::gravityMode()
{
	m_FlowControl->e_gravityMode();
}

void YouBot_executive::positionArm(vector<double> position_j)
{
	//RTT::log(Info) << "Call positionArm" << endlog();
	m_position_j.assign(position_j.begin(),position_j.end());
	m_FlowControl->e_jointControl();

}

void YouBot_executive::positionGripper(vector<double> position_c)
{
	//RTT::log(Info) << "Call positionGripper" << endlog();
	m_position_c.assign(position_c.begin(),position_c.end());
	m_FlowControl->e_catesianControl();
}

void YouBot_executive::writeSetpoints(const vector<double> & position_j,
		const vector<double> & stiffness_j, const vector<double> & position_c,
		const vector<double> & stiffness_c)
{
	if ( position_j.size()==0 || stiffness_j.size()==0 ||
			position_c.size()==0|| stiffness_c.size()<2)
	{
		RTT::log(Error) << "Attempt to write strange value to the port, executive state is "<<m_FlowControl->toString()<< endlog();
		this->error();
		return;
	}
	msgs_setpoint_j.data.assign(position_j.begin(), position_j.end());
	msgs_stiffness_j.data.assign(stiffness_j.begin(), stiffness_j.end());
	msgs_setpoint_c.data.assign(position_c.begin(), position_c.end());
	msgs_stiffness_c_r.data.at(0)=stiffness_c.at(0);
	msgs_stiffness_c_t.data.at(0)=stiffness_c.at(1);
	m_JointSpaceSetpoint.write(msgs_setpoint_j);
	m_JointSpaceStiffness.write(msgs_stiffness_j);
	m_CartSpaceSetpoint.write(msgs_setpoint_c);
	m_CartSpaceStiffness_r.write(msgs_stiffness_c_r);
	m_CartSpaceStiffness_t.write(msgs_stiffness_c_t);
}

void YouBot_executive::getArmPose(vector<double> & position_j)
{
	//RTT::log(Info) << "Call getArmPose" << endlog();
	std_msgs::Float64MultiArray sample;
	if (m_JointGripperPose.read(sample) != RTT::NoData)
	{
		position_j.assign(sample.data.begin(), sample.data.end());
	}
}

void YouBot_executive::getGripperPose(vector<double> & position_c)
{
	//RTT::log(Info) << "Call getGripperPose" << endlog();
	std_msgs::Float64MultiArray sample;
	if (m_CartGripperPose.read(sample) != RTT::NoData)
	{
		getXYZYPR(sample.data, position_c);
	}
}

void YouBot_executive::getXYZYPR(const vector<double> & H,
		vector<double> & XYZYPR)
{
	XYZYPR.clear();
	XYZYPR.resize(6);
	XYZYPR.at(0) = H.at(3);
	XYZYPR.at(1) = H.at(7);
	XYZYPR.at(2) = H.at(11);
	btMatrix3x3 rotMatrix(H[0], H[1], H[2], H[4], H[5], H[6], H[8], H[9],
			H[10]);
	btScalar y, p, r;
	rotMatrix.getEulerYPR(y, p, r);
	XYZYPR.at(3) = (double) (y);
	XYZYPR.at(4) = (double) (p);
	XYZYPR.at(5) = (double) (r);
}
void YouBot_executive::guardMove(vector<double> force_c)
{
	m_force_c.assign(force_c.begin(),force_c.end());
	m_FlowControl->e_guardedMove();
}
void YouBot_executive::getGuardForce(vector<double>& force_c)
{
	force_c.assign(m_force_c.begin(),m_force_c.end());
}
void YouBot_executive::getStateForce(vector<double>& force_c)
{
	std_msgs::Float64MultiArray sample;
	force_c.resize(SIZE_CART_SPACE);
	if (m_CartForceState.read(sample) != RTT::NoData)
	{
		if(sample.data.size()==SIZE_CART_SPACE)
		{
			force_c.at(0)=sample.data.at(3); //x
			force_c.at(1)=sample.data.at(4); //y
			force_c.at(2)=sample.data.at(5); //z
			force_c.at(3)=sample.data.at(0); //omega_x
			force_c.at(4)=sample.data.at(1); //omega_y
			force_c.at(5)=sample.data.at(2); //omega_z
		}
	}
}

void YouBot_executive::doneEvent(){
	std::string msgs("executive.e_done");
	m_events.write(msgs);
}
}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)
