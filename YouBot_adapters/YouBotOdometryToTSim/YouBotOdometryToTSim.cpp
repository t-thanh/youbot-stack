#include "YouBotOdometryToTSim.hpp"

#include <iostream>
#include <ocl/Component.hpp>
#include <tf/tf.h>

/*
 * @brief Adapter to connect the YouBot OODL to 20Sim.
 */
namespace YouBot
{
	using namespace RTT;

	YouBotOdometryToTSim::YouBotOdometryToTSim(string const& name) :
			TaskContext(name),
			m_dimension(3)
	{
		m_output_odometry.data = vector<double>(0);

		this->addPort("output_odometry",output_odometry)
			.doc("Connect 20Sim flat vectors. Converts YouBot odometry  messages to positions vectors.");


		this->addEventPort("input_odometry",input_odometry)
			.doc("Connect YouBot::OODL odometry messages. This is an event port.");

		this->addOperation("initialize",&YouBotOdometryToTSim::initialize,this, OwnThread);
	}

	YouBotOdometryToTSim::~YouBotOdometryToTSim() {}

	void YouBotOdometryToTSim::initialize()
	{
		m_dimension = 3;

		m_output_odometry.data.resize(m_dimension, 0.0);

		output_odometry.setDataSample( m_output_odometry );
	}

	bool YouBotOdometryToTSim::startHook()
	{
		if(!( output_odometry.connected() ) )
		{
			log(Error) << "The the output ports needs to be connected." << endlog();
			return false;
		}

		if(! input_odometry.connected() )
		{
			log(Error) << "The input_odometry need to be connected." << endlog();
			return false;
		}

		if(m_dimension == 0)
		{
			log(Error) << "The dimension of the input/output signals needs to be greater than 0, use initialize()." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void YouBotOdometryToTSim::updateHook()
	{
		TaskContext::updateHook();
		if(input_odometry.read(m_input_odometry) == NewData)
		{

			m_output_odometry.data.at(0)=tf::getYaw(m_input_odometry.pose.pose.orientation);
			m_output_odometry.data.at(1)=m_input_odometry.pose.pose.position.x;
			m_output_odometry.data.at(2)=m_input_odometry.pose.pose.position.y;
			output_odometry.write(m_output_odometry);
		}
	}
}

ORO_CREATE_COMPONENT(YouBot::YouBotOdometryToTSim)
