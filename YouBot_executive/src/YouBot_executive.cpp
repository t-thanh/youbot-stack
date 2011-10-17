#include "YouBot_executive.h"

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

YouBot_executive::YouBot_executive(const string& name) :
		TaskContext(name, PreOperational)
{
	this->addOperation("unfoldArm",&YouBot_executive::unfoldArm,this).doc("jut text");
	this->addPort("JointSpaceSetpoint",JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffnes",JointSpaceStiffnes).doc("");

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
	double d_stiffnes[8]= {0,0,0,1,1,1,1,1};
	msgs_setpoint.data.assign(d_setpoint,d_setpoint+8);
	msgs_stiffnes.data.assign(d_stiffnes,d_stiffnes+8);
	JointSpaceSetpoint.write(msgs_setpoint);
	JointSpaceStiffnes.write(msgs_stiffnes);
}
}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)
