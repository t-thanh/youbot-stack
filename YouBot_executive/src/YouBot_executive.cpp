#include "YouBot_executive.h"

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

YouBot_executive::YouBot_executive(const string& name) :
		TaskContext(name, PreOperational), m_position_j(),m_stiffness_j(),m_position_c(),m_stiffness_c()
{
	m_position_j.assign(8,0);
	m_stiffness_j.assign(8,0);
	m_position_c.assign(6,0);
	m_stiffness_c.assign(2,0);
	//Operation<void(void)> op_unfoldArm("unfoldArm",&YouBot_executive::unfoldArm,this);
	this->addOperation("unfoldArm",&YouBot_executive::unfoldArm,this).doc("jut text");

	//Operation<void(void)> op_gravityMode("gravityMode",&YouBot_executive::gravityMode,OwnThread);
	this->addOperation("gravityMode",&YouBot_executive::gravityMode,this).doc("jut text");

	//Operation<void(void)> op_positionArm("positionArm",&YouBot_executive::positionArm,OwnThread);
	this->addOperation("positionArm",&YouBot_executive::positionArm,this).doc("jut text");
	this->addOperation("positionGripper",&YouBot_executive::positionGripper,this).doc("jut text");

	this->addPort("JointSpaceSetpoint",m_JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffness",m_JointSpaceStiffness).doc("");
	this->addPort("CartSpaceSetpoint",m_CartSpaceSetpoint).doc("");
	this->addPort("CartSpaceStiffness_rot",m_CartSpaceStiffness_r).doc("");
	this->addPort("CartSpaceStiffness_trans",m_CartSpaceStiffness_t).doc("");

	this->addProperty("Joint_position_setpoint",m_position_j);
	this->addProperty("Joint_stiffness_setpoint",m_stiffness_j);
	this->addProperty("Gripper_position_setpoint",m_position_c);
	this->addProperty("Gripper_stiffness_setpoint",m_stiffness_c);

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
	double d_setpoint_j[8]= {0,0,0,0,0,0,0,0};
	double d_stiffness_j[8]= {0,0,0,5,5,5,5,5};
	vector<double> v_setpoint_j,v_stiffness_j;
	v_setpoint_j.assign(d_setpoint_j,d_setpoint_j+8);
	v_stiffness_j.assign(d_stiffness_j,d_stiffness_j+8);

	double d_setpoint_c[6]= {0,0,1,0,0,0,};
	double d_stiffness_c[2]= {10,0};
	vector<double> v_setpoint_c,v_stiffness_c;
	v_setpoint_c.assign(d_setpoint_c,d_setpoint_c+6);
	v_stiffness_c.assign(d_stiffness_c,d_stiffness_c+2);
	writeSetpoints(v_setpoint_j,v_stiffness_j,v_setpoint_c,v_stiffness_c);
}
void YouBot_executive::gravityMode()
{
	double d_setpoint_j[8]= {0,0,0,0,0,0,0,0};
	double d_stiffness_j[8]= {0,0,0,0,0,0,0,0};
	vector<double> v_setpoint_j,v_stiffness_j;
	v_setpoint_j.assign(d_setpoint_j,d_setpoint_j+8);
	v_stiffness_j.assign(d_stiffness_j,d_stiffness_j+8);

	double d_setpoint_c[6]= {0,0,1,0,0,0,};
	double d_stiffness_c[2]= {0,0};
	vector<double> v_setpoint_c,v_stiffness_c;
	v_setpoint_c.assign(d_setpoint_c,d_setpoint_c+6);
	v_stiffness_c.assign(d_stiffness_c,d_stiffness_c+2);
	writeSetpoints(v_setpoint_j,v_stiffness_j,v_setpoint_c,v_stiffness_c);
}
void YouBot_executive::positionArm()
{

	double d_setpoint_c[6]= {0,0,1,0,0,0,};
	double d_stiffness_c[2]= {0,0};
	vector<double> v_setpoint_c,v_stiffness_c;
	v_setpoint_c.assign(d_setpoint_c,d_setpoint_c+6);
	v_stiffness_c.assign(d_stiffness_c,d_stiffness_c+2);
	writeSetpoints(vector<double>(),vector<double>(),v_setpoint_c,v_stiffness_c);
}
void YouBot_executive::positionGripper()
{
	double d_setpoint_j[8]= {0,0,0,0,0,0,0,0};
	double d_stiffness_j[8]= {0,0,0,0,0,0,0,0};
	vector<double> v_setpoint_j,v_stiffness_j;
	v_setpoint_j.assign(d_setpoint_j,d_setpoint_j+8);
	v_stiffness_j.assign(d_stiffness_j,d_stiffness_j+8);
	writeSetpoints(v_setpoint_j,v_stiffness_j,vector<double>(),vector<double>());
}
void YouBot_executive::writeSetpoints(const vector<double>& position_j,const vector<double>& stiffness_j,
		const vector<double>& position_c, const vector<double>& stiffness_c)
{
	std_msgs::Float64MultiArray msgs_setpoint_j;
	 if(position_j.size()==0)
		msgs_setpoint_j.data.assign(m_position_j.begin(),m_position_j.end());
	else
		msgs_setpoint_j.data.assign(position_j.begin(),position_j.end());

	std_msgs::Float64MultiArray msgs_stiffness_j;
	 if(stiffness_j.size()==0)
		msgs_stiffness_j.data.assign(m_stiffness_j.begin(),m_stiffness_j.end());
	else
		msgs_stiffness_j.data.assign(stiffness_j.begin(),stiffness_j.end());

	std_msgs::Float64MultiArray msgs_setpoint_c;
 if(position_c.size()==0)
		msgs_setpoint_c.data.assign(m_position_c.begin(),m_position_c.end());
	else
		msgs_setpoint_c.data.assign(position_c.begin(),position_c.end());

	std_msgs::Float64MultiArray msgs_stiffness_c_r;
	std_msgs::Float64MultiArray msgs_stiffness_c_t;
	msgs_stiffness_c_r.data.clear();
	msgs_stiffness_c_t.data.clear();
 if(stiffness_c.size()<2)
	{
		msgs_stiffness_c_r.data.push_back(m_stiffness_c.at(0));
		msgs_stiffness_c_t.data.push_back(m_stiffness_c.at(1));
	}
	else
	{
		msgs_stiffness_c_r.data.push_back(stiffness_c.at(0));
		msgs_stiffness_c_t.data.push_back(stiffness_c.at(1));
	}




	m_JointSpaceSetpoint.write(msgs_setpoint_j);
	m_JointSpaceStiffness.write(msgs_stiffness_j);
	m_CartSpaceSetpoint.write(msgs_setpoint_c);
	m_CartSpaceStiffness_r.write(msgs_stiffness_c_r);
	m_CartSpaceStiffness_t.write(msgs_stiffness_c_t);
}
}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)
