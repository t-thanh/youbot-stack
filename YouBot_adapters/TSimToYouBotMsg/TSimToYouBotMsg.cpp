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

		this->addEventPort("input_cmd_signal",input_cmd_signal)
			.doc("Connect 20Sim controller signal. This is an event port.");

		this->addOperation("initialize",&TSimToYouBotMsg::initialize,this, OwnThread);

		//RTT::types::Types()->addType( new RTT::types::SequenceTypeInfo<std::vector<double> >("std.vector<double>") );
	}

	TSimToYouBotMsg::~TSimToYouBotMsg() {}

	void TSimToYouBotMsg::initialize(ctrl_modes ctrl_mode, unsigned int dimension)
	{
		if(ctrl_mode == MOTOR_STOP)
		{
			log(Error) << "ctrl_mode cannot be MOTOR_STOP for this component." << endlog();
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

		return true;
	}

	void TSimToYouBotMsg::updateHook()
	{
		if(input_cmd_signal.read(m_input_cmd_signal) == NewData)
		{
			switch(m_ctrl_mode)
			{
				case(PLANE_ANGLE):
				{
					for(unsigned int i = 0; i < m_dimension; ++i)
					{
						m_output_cmd_angles.positions[i] = m_input_cmd_signal.data[i];
					}
				}
				case(ANGULAR_VELOCITY):
				{
					for(unsigned int i = 0; i < m_dimension; ++i)
					{
						m_output_cmd_velocities.velocities[i] = m_input_cmd_signal.data[i];
					}
				}
				case(TORQUE):
				{
					for(unsigned int i = 0; i < m_dimension; ++i)
					{
						m_output_cmd_torques.efforts[i] = m_input_cmd_signal.data[i];
					}
				}
				default:
				{
					this->error();
				}
			}
		}
	}
}

ORO_CREATE_COMPONENT( YouBot::TSimToYouBotMsg )
