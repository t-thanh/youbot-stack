#include "TSimToYouBotMsg.hpp"

#include <iostream>
#include <ocl/Component.hpp>
//#include <rtt/types/SequenceTypeInfo.hpp>

/*
 * @brief Adapter to connect 20Sim to the YouBot OODL.
 */
namespace YouBot
{
	using namespace RTT;

	TSimToYouBotMsg::TSimToYouBotMsg(string const& name) :
			TaskContext(name),
			m_ctrl_mode(MOTOR_STOP),
			m_dimension(0)
	{
		m_input_cmd_signal.data = vector<double>(0); //TODO: Fix me properly
		this->addPort("output_cmd_angles",output_cmd_angles)
			.doc("Connect OODL angles. Converts 20Sim vectors to YouBot cmd messages.");
		this->addPort("output_cmd_velocities",output_cmd_velocities)
			.doc("Connect OODL velocities. Converts 20Sim vectors to YouBot cmd messages.");
		this->addPort("output_cmd_torques",output_cmd_torques)
			.doc("Connect OODL torques. Converts 20Sim vectors to YouBot cmd messages.");
    this->addPort("output_cmd_twist",output_cmd_twist)
      .doc("Connect OODL twist. Converts 20Sim vectors to YouBot cmd messages.");

		this->addEventPort("input_cmd_signal",input_cmd_signal)
			.doc("Connect 20Sim controller signal. This is an event port.");

		this->addOperation("initialize",&TSimToYouBotMsg::initialize,this, OwnThread);

	}

	TSimToYouBotMsg::~TSimToYouBotMsg() {}

	void TSimToYouBotMsg::initialize(ctrl_modes ctrl_mode, unsigned int dimension)
	{
		log(Info) << "Setting up with ctrl_mode: " << ctrl_mode << " and dimension: " << dimension << endlog();
		if(ctrl_mode == MOTOR_STOP)
		{
			log(Error) << "ctrl_mode cannot be MOTOR_STOP for this component." << endlog();
		}

		if(ctrl_mode == TWIST && dimension != 6)
		{
			log(Error) << "ctrl_mode TWIST expects a 6 dimensional vector." << endlog();
		}

		m_ctrl_mode = ctrl_mode;
		m_dimension = dimension;

		m_input_cmd_signal.data.resize(m_dimension, 0);

		m_output_cmd_angles.positions.assign(m_dimension, 0);
		m_output_cmd_velocities.velocities.assign(m_dimension, 0);
		m_output_cmd_torques.efforts.assign(m_dimension, 0);

		output_cmd_angles.setDataSample( m_output_cmd_angles );
		output_cmd_velocities.setDataSample( m_output_cmd_velocities );
		output_cmd_torques.setDataSample( m_output_cmd_torques );
		output_cmd_twist.setDataSample(m_output_cmd_twist);
	}

	bool TSimToYouBotMsg::startHook()
	{
		if(m_ctrl_mode == PLANE_ANGLE && ! output_cmd_angles.connected())
		{
			log(Error) << "The constructor ctrl_mode must match the connected output_cmd_* port." << endlog();
			return false;
		}
		else if(m_ctrl_mode == ANGULAR_VELOCITY && ! output_cmd_velocities.connected())
		{
			log(Error) << "The constructor ctrl_mode must match the connected output_cmd_* port." << endlog();
			return false;
		}
		else if(m_ctrl_mode == TORQUE && ! output_cmd_torques.connected())
		{
			log(Error) << "The constructor ctrl_mode must match the connected output_cmd_* port." << endlog();
			return false;
		}
		else if(m_ctrl_mode == TWIST && ! output_cmd_twist.connected())
		{
			log(Error) << "The constructor ctrl_mode must match the connected output_cmd_* port." << endlog();
			return false;
		}
		else if(m_ctrl_mode == MOTOR_STOP)
		{
			log(Error) << "Atleast one of the output_cmd_* ports needs to be connected." << endlog();
			log(Error) << "Furthermore, the following ctrl_modes are supported: PLANE_ANGLE, ANGULAR_VELOCITY and TORQUE." << endlog();
			return false;
		}

		if(! input_cmd_signal.connected() )
		{
			log(Error) << "The input_states need to be connected." << endlog();
			return false;
		}

		if(m_dimension == 0)
		{
			log(Error) << "The dimension of the input/output signals needs to be greater than 0, use initialize()." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void TSimToYouBotMsg::updateHook()
	{
		TaskContext::updateHook();
		if(input_cmd_signal.read(m_input_cmd_signal) == NewData)
		{
			//log(Info) << "NewData" << endlog();
			switch(m_ctrl_mode)
			{
				case(PLANE_ANGLE):
				{
					for(unsigned int i = 0; i < m_input_cmd_signal.data.size(); ++i)
					{
						m_output_cmd_angles.positions[i] = m_input_cmd_signal.data[i];
					}
					output_cmd_angles.write(m_output_cmd_angles);
					break;
				}
				case(ANGULAR_VELOCITY):
				{
					for(unsigned int i = 0; i < m_input_cmd_signal.data.size(); ++i)
					{
						m_output_cmd_velocities.velocities[i] = m_input_cmd_signal.data[i];
					}
					output_cmd_velocities.write(m_output_cmd_velocities);
					break;
				}
				case(TORQUE):
				{
					for(unsigned int i = 0; i < m_input_cmd_signal.data.size(); ++i)
					{
						m_output_cmd_torques.efforts[i] = m_input_cmd_signal.data[i];
					}
					output_cmd_torques.write(m_output_cmd_torques);
					break;
				}
				case(TWIST):
				{
					if(m_input_cmd_signal.data.size() != 6)
					{
						this->error();
						break;
					}
					m_output_cmd_twist.linear.x = m_input_cmd_signal.data[0];
					m_output_cmd_twist.linear.y = m_input_cmd_signal.data[1];
					m_output_cmd_twist.linear.z = m_input_cmd_signal.data[2];

					m_output_cmd_twist.angular.x = m_input_cmd_signal.data[3];
					m_output_cmd_twist.angular.y = m_input_cmd_signal.data[4];
					m_output_cmd_twist.angular.z = m_input_cmd_signal.data[5];

					output_cmd_twist.write(m_output_cmd_twist);
					break;
				}
				default:
				{
					this->error();
					break;
				}
			}
		}
	}
}

ORO_CREATE_COMPONENT( YouBot::TSimToYouBotMsg )
