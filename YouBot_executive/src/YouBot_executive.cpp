#include "YouBot_executive.h"

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

YouBot_executive::YouBot_executive(const string& name) :
		TaskContext(name, PreOperational), position(),stiffness()
{
	position.assign(8,0);
	stiffness.assign(8,0);
	Operation<void(void)> op_unfoldArm("unfoldArm",&YouBot_executive::unfoldArm,this);
	this->addOperation(op_unfoldArm).doc("jut text");

	Operation<void(void)> op_gravityMode("gravityMode",&YouBot_executive::gravityMode,this);
	this->addOperation(op_gravityMode).doc("jut text");

	Operation<void(void)> op_positionArm("positionArm",&YouBot_executive::positionArm,this);
	this->addOperation(op_positionArm).doc("jut text");

	this->addPort("JointSpaceSetpoint",JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffness",JointSpaceStiffness).doc("");
	this->addProperty("Position_setpoint",position);
	this->addProperty("Stiffness_setpoint",stiffness);

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

}

void YouBot_executive::stopHook()
{

	TaskContext::stopHook();
}

void YouBot_executive::cleanupHook()
{
	TaskContext::cleanupHook();
}
void YouBot_executive::unfoldArm()
{
	RTT::log(RTT::Fatal)<<"it is success"<<RTT::endlog();
	std_msgs::Float64MultiArray msgs_setpoint;
	std_msgs::Float64MultiArray msgs_stiffnes;
	double d_setpoint[8]= {0,0,0,0,0,0,0,0};
	double d_stiffnes[8]= {0,0,0,5,5,5,5,5};
	msgs_setpoint.data.assign(d_setpoint,d_setpoint+8);
	msgs_stiffnes.data.assign(d_stiffnes,d_stiffnes+8);
	JointSpaceSetpoint.write(msgs_setpoint);
	JointSpaceStiffness.write(msgs_stiffnes);
}
void YouBot_executive::gravityMode()
{
	RTT::log(RTT::Fatal)<<"it is success"<<RTT::endlog();
	std_msgs::Float64MultiArray msgs_setpoint;
	std_msgs::Float64MultiArray msgs_stiffnes;
	double d_setpoint[8]= {0,0,0,0,0,0,0,0};
	double d_stiffnes[8]= {0,0,0,0,0,0,0,0};
	msgs_setpoint.data.assign(d_setpoint,d_setpoint+8);
	msgs_stiffnes.data.assign(d_stiffnes,d_stiffnes+8);
	JointSpaceSetpoint.write(msgs_setpoint);
	JointSpaceStiffness.write(msgs_stiffnes);
}
void YouBot_executive::positionArm()
{
	RTT::log(RTT::Fatal)<<"it is success"<<RTT::endlog();
	std_msgs::Float64MultiArray msgs_setpoint;
	std_msgs::Float64MultiArray msgs_stiffness;
	msgs_setpoint.data.assign(position.begin(),position.end());
	msgs_stiffness.data.assign(stiffness.begin(),stiffness.end());
	JointSpaceSetpoint.write(msgs_setpoint);
	JointSpaceStiffness.write(msgs_stiffness);
}
}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)
