#include "YouBotMonitorService.hpp"

#include <rtt/plugin/ServicePlugin.hpp>

#include <stdio.h>
#include <cassert>

#include <boost/lexical_cast.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	/**
	 * @brief Generic services for YouBot.
	 */
	YouBotMonitorService::YouBotMonitorService(TaskContext* parent) :
			Service("YouBotMonitorService", parent)
	{
		memset(&m_events.stamp, 0, sizeof(ros::Time));
		m_events.monitor_event.reserve(max_event_length);

        // Pre-allocate port memory for outputs
		events.setDataSample(m_events);

		setupComponentInterface();
	}

	YouBotMonitorService::~YouBotMonitorService()
	{ }

	void YouBotMonitorService::setupComponentInterface()
	{
		this->addPort("events", events).doc("Joint events");
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message)
	{
		m_events.stamp = ros::Time::now();
		m_events.monitor_event = id + "." + message; //"jnt" + boost::lexical_cast<string>(joint)
		events.write(m_events);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message, bool condition)
	{
		m_events.stamp = ros::Time::now();
		m_events.monitor_event = id + "." + message + "_" + (condition ? "true" : "false");
		events.write(m_events);
	}

	void YouBotMonitorService::update()
	{
		unsigned int size = m_monitors.size();
		for(unsigned int i = 0; i < size; ++i)
		{
			if(m_monitors[i].check())
			{
				if(m_monitors[i].e_type == EDGE && !m_monitors[i].state)
				{
					emitEvent(m_monitors[i].id, m_monitors[i].msg, true);
				}
				else if(m_monitors[i].e_type == LEVEL)
				{
					emitEvent(m_monitors[i].id, m_monitors[i].msg);
				}
			}
			else if(m_monitors[i].e_type == EDGE && m_monitors[i].state)
			{
				emitEvent(m_monitors[i].id, m_monitors[i].msg, false);
			}
		}
	}

	bool YouBotMonitorService::bind_function(monitor& m)
	{
		// Note: You MUST use the final (heap stored) monitor struct!

		if(m.space == CARTESIAN && m.part == BASE)
		{
			m.check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<nav_msgs::Odometry>), this, &base_cart_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
		}
		else if(m.space == CARTESIAN && m.part == ARM)
		{
//			m.check = boost::bind(&YouBotMonitorService::check_monitor, this, &arm_cart_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
			log(Error) << "Not implemented!" << endlog();
			return false;
		}
		else if(m.space == JOINT && m.part == BASE)
		{
			m.check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &base_joint_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
		}
		else if(m.space == JOINT && m.part == ARM)
		{
			m.check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &arm_joint_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
		}

		return true;
	}

	void YouBotMonitorService::setup_monitor(physical_part& part, control_space& space, physical_quantity& quantity,
			event_type& e_type, compare_type& c_type, std::string& msg, unsigned int& index, double& single_value)
	{
		monitor m;
		m.part = part;
		m.space = space;
		m.quantity = quantity;
		m.e_type = e_type;
		m.c_type = c_type;
		m.msg = msg;

		m.is_single_value = true;
		m.indices.resize(1, index);
		m.values.resize(1, single_value);

		if(m.space == CARTESIAN && (m.quantity == FORCE || m.quantity == TORQUE) )
		{
			log(Error) << "Cannot monitor cartesian FORCE or TORQUE at the moment." << endlog();
			this->getOwner()->error();
			return;
		}

		if(m.space == CARTESIAN)
		{
			m.id = control_space_tostring(m.space) + physical_quantity_tostring(m.quantity);
		}
		else
		{
			m.id = control_space_tostring(m.space) + physical_quantity_tostring(m.quantity) + boost::lexical_cast<string>(index);
		}

		m_monitors.push_back(m);

		if(!bind_function(m_monitors[m_monitors.size() -1]))
		{
			m_monitors.pop_back();
		}
	}

	void YouBotMonitorService::setup_monitor(physical_part& part, control_space& space, physical_quantity& quantity,
			event_type& e_type, compare_type& c_type, std::string& msg, vector<unsigned int>& indices, vector<double>& values)
	{
		monitor m;
		m.part = part;
		m.space = space;
		m.quantity = quantity;
		m.e_type = e_type;
		m.c_type = c_type;
		m.msg = msg;

		m.is_single_value = false;
		m.indices = indices;
		m.values = values;

		if(m.space == CARTESIAN && (m.quantity == FORCE || m.quantity == TORQUE) )
		{
			log(Error) << "Cannot monitor cartesian FORCE or TORQUE at the moment." << endlog();
			this->getOwner()->error();
			return;
		}

		if(m.space == CARTESIAN)
		{
			m.id = control_space_tostring(m.space) + physical_quantity_tostring(m.quantity);
		}
		else
		{
			m.id.reserve(13);
			m.id = control_space_tostring(m.space);
			for(unsigned int i = 0; i < indices.size(); ++i)
			{
				m.id.append( boost::lexical_cast<string>(indices[i]));
			}
			m.id.append(physical_quantity_tostring(m.quantity));
		}

		m_monitors.push_back(m);

		if(!bind_function(m_monitors[m_monitors.size() -1]))
		{
			m_monitors.pop_back();
		}
	}

	ORO_SERVICE_NAMED_PLUGIN(YouBotMonitorService, "YouBotMonitorService")
}
