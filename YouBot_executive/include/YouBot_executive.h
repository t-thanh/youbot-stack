#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
using namespace RTT;
typedef std_msgs::Float64MultiArray flat_matrix_t;
class YouBot_executive: public TaskContext
{
public:
	YouBot_executive(const string& name);
	virtual ~YouBot_executive();
	void unfoldArm();
	void gravityMode();
	void positionArm();
	void positionGripper();
	vector<double> m_position_j;
	vector<double> m_stiffness_j;
	vector<double> m_position_c;
	vector<double> m_stiffness_c;
	RTT::OutputPort<flat_matrix_t> m_JointSpaceSetpoint;
	RTT::OutputPort<flat_matrix_t> m_JointSpaceStiffness;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceSetpoint;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceStiffness_r;
	RTT::OutputPort<flat_matrix_t> m_CartSpaceStiffness_t;
protected:
	void writeSetpoints(const vector<double>& position_j,const vector<double>& stiffness_j,
			const vector<double>& position_c, const vector<double>& stiffness_c);
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

};
}
