#include "YouBotMsgToTSim.hpp"

#include <iostream>
#include <ocl/Component.hpp>

/*
 * @brief Adapter to connect the YouBot OODL to 20Sim.
 */
namespace YouBot
{
	using namespace RTT;

	YouBotMsgToTSim::YouBotMsgToTSim(string const& name) :
			TaskContext(name),
			m_dimension(0)
	{
		m_output_positions.data = vector<double>(0);
		m_output_velocities.data = vector<double>(0);
		m_output_torques.data = vector<double>(0);

		this->addPort("output_positions",output_positions)
			.doc("Connect 20Sim flat vectors. Converts YouBot state messages to positions vectors.");
		this->addPort("output_velocities",output_velocities)
			.doc("Connect 20Sim flat vectors. Converts YouBot state messages to velocity vectors.");
		this->addPort("output_torques",output_torques)
			.doc("Connect 20Sim flat vectors. Converts YouBot state messages to torque vectors.");

		this->addEventPort("input_states",input_states)
			.doc("Connect YouBot::OODL state messages. This is an event port.");

		this->addOperation("initialize",&YouBotMsgToTSim::initialize,this, OwnThread);
	}

	YouBotMsgToTSim::~YouBotMsgToTSim() {}

	void YouBotMsgToTSim::initialize(unsigned int dimension)
	{
		m_dimension = dimension;

		m_output_positions.data.resize(m_dimension, 0.0);
		m_output_velocities.data.resize(m_dimension, 0.0);
		m_output_torques.data.resize(m_dimension, 0.0);

		m_input_states.position.assign(m_dimension, 0);
		m_input_states.velocity.assign(m_dimension, 0);
		m_input_states.effort.assign(m_dimension, 0);

		output_positions.setDataSample( m_output_positions );
		output_velocities.setDataSample( m_output_velocities );
		output_torques.setDataSample( m_output_torques );
	}

	bool YouBotMsgToTSim::startHook()
	{
		if(!( output_positions.connected() || output_velocities.connected() || !output_torques.connected() ) )
		{
			log(Error) << "Atleast one of the output_* ports needs to be connected." << endlog();
			return false;
		}

		if(! input_states.connected() )
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

	void YouBotMsgToTSim::updateHook()
	{
		TaskContext::updateHook();
		if(input_states.read(m_input_states) == NewData)
		{
			for(unsigned int i = 0; i < m_dimension; ++i)
			{
				m_output_positions.data[i] = m_input_states.position[i];
				m_output_velocities.data[i] = m_input_states.velocity[i];
				m_output_torques.data[i] = m_input_states.effort[i];
			}
			output_positions.write(m_output_positions);
			output_velocities.write(m_output_velocities);
			output_torques.write(m_output_torques);
		}
	}
}

ORO_CREATE_COMPONENT(YouBot::YouBotMsgToTSim)
