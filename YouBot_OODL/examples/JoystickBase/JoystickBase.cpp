#include "JoystickBase.hpp"
#include "YouBotOODL.hpp"

#include <ocl/Component.hpp>

#include <vector>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace boost::units;
	using namespace boost::units::si;

	JoystickBase::JoystickBase(const string& name) :
			TaskContext(name, PreOperational)
	{
		memset(&m_error, 0, sizeof(double[3]));
		m_K[0] = 0.5; m_K[1] = 0.5; m_K[2] = 0.5;

//		memset(&m_odometry_state, 0, sizeof(m_odometry_state));
//		memset(&m_cmd_twist, 0, sizeof(m_cmd_twist));

		cmd_twist.setDataSample(m_cmd_twist);

		this->addPort("odometry_state" , odometry_state).doc("Input odometry state from the driver.");
		this->addPort("cmd_twist", cmd_twist).doc("Output twist commands to the driver.");
		this->addPort("joystick", joystick).doc("Input from joystick node.");
	}

	JoystickBase::~JoystickBase() {}

	bool JoystickBase::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool JoystickBase::startHook()
	{
		if(!odometry_state.connected() || !cmd_twist.connected() || !joystick.connected() )
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		TaskContext* task_ptr = getPeer("OODL");
		if(task_ptr == NULL)
		{
			log(Error) << "Could not find peer OODL" << endlog();
			return false;
		}
		op_setControlModes = task_ptr->provides("Base")->getOperation("setControlModes");

		if(!op_setControlModes.ready())
		{
			log(Error) << "Could not connect to Base services" << endlog();
			return false;
		}

		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, TWIST);
		op_setControlModes(m_modes);

		log(Info) << "startHook done." << endlog();

		return TaskContext::startHook();
	}

	void JoystickBase::updateHook()
	{
        TaskContext::updateHook();

        if(joystick.read(m_joystick) == NoData || m_joystick.axes.size() < 3)
        {
//        	log(Error) << "Cannot read the joystick" << endlog();
        	return;
        }

        if(odometry_state.read(m_odometry_state) == NoData)
        {
//        	log(Error) << "Cannot read the odometry state." << endlog();
        	return;
        }

        // Do control loop!
        m_error[0] = m_odometry_state.twist.twist.linear.x + m_joystick.axes[1]; // x
        m_error[1] = m_odometry_state.twist.twist.linear.y + m_joystick.axes[0]; // y
        m_error[2] = m_odometry_state.twist.twist.angular.z + m_joystick.axes[2]; // yaw

        m_cmd_twist.linear.x = m_K[0] * m_error[0];
        m_cmd_twist.linear.y = m_K[1] * m_error[1];
        m_cmd_twist.angular.z = m_K[2] * m_error[2];

        cmd_twist.write(m_cmd_twist);
	}

	void JoystickBase::stopHook()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, MOTOR_STOP);
		op_setControlModes(m_modes);

        TaskContext::stopHook();
	}

	void JoystickBase::cleanupHook()
	{
        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::JoystickBase )
