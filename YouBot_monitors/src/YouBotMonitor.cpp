#include "YouBotMonitor.hpp"

#include <rtt/plugin/ServicePlugin.hpp>
#include <ocl/Component.hpp>

#include <stdio.h>
#include <cassert>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>


namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	/**
	 * @brief Generic services for YouBot.
	 */
	YouBotMonitorService::YouBotMonitorService(const string& name) :
			TaskContext(name)
	{
		m_events.data.reserve(max_event_length);
		m_events.data.assign("");

        // Pre-allocate port memory for outputs
		events.setDataSample(m_events);

		setupComponentInterface();
	}

	YouBotMonitorService::~YouBotMonitorService()
	{ }

	void YouBotMonitorService::setupComponentInterface()
	{
		this->addPort("events", events).doc("Joint events");

		this->addPort("arm_joint_state", arm_joint_state).doc("Arm joint states");
		this->addPort("base_joint_state", base_joint_state).doc("Base joint states");
		this->addPort("base_cart_state", base_cart_state).doc("Base cartesian states");

        this->addOperation("setup_monitor",&YouBotMonitorService::setup_monitor,this, OwnThread);
        this->addOperation("activate_monitor",&YouBotMonitorService::activate_monitor,this, OwnThread);
        this->addOperation("deactivate_monitor",&YouBotMonitorService::deactivate_monitor,this, OwnThread);
//        this->addOperation("remove_monitors",&YouBotMonitorService::remove_monitors,this, OwnThread);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message)
	{
		m_events.data = id + "." + message; //"jnt" + boost::lexical_cast<string>(joint)
		events.write(m_events);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message, bool condition)
	{
		m_events.data = id + "." + message + "_" + (condition ? "true" : "false");
		events.write(m_events);
	}

	bool YouBotMonitorService::startHook()
	{
		if(! (base_joint_state.connected() && base_cart_state.connected() && arm_joint_state.connected()) )
		{
			return false;
		}
		return true;
	}

	void YouBotMonitorService::updateHook()
	{
		base_joint_state.read(m_base_joint_state);
		base_cart_state.read(m_base_cart_state);
		arm_joint_state.read(m_arm_joint_state);

		unsigned int size = m_active_monitors.size();
		for(unsigned int i = 0; i < size; ++i)
		{
			if(m_active_monitors[i]->check())
			{
				if(m_active_monitors[i]->e_type == EDGE && !m_active_monitors[i]->state)
				{
					m_active_monitors[i]->state = true;
					emitEvent(m_active_monitors[i]->id, m_active_monitors[i]->msg, true);
				}
				else if(m_active_monitors[i]->e_type == LEVEL)
				{
					emitEvent(m_active_monitors[i]->id, m_active_monitors[i]->msg);
				}
			}
			else if(m_active_monitors[i]->e_type == EDGE && m_active_monitors[i]->state)
			{
				m_active_monitors[i]->state = false;
				emitEvent(m_active_monitors[i]->id, m_active_monitors[i]->msg, false);
			}
		}
	}

	vector<monitor*>::iterator YouBotMonitorService::getMonitor(vector<monitor*>& list, std::string& name)
	{
		for(vector<monitor*>::iterator i = list.begin(); i < list.end(); ++i)
		{
			if(boost::iequals(name, (*i)->descriptive_name))
			{
				return i;
			}
		}
		return list.end();
	}

	void YouBotMonitorService::setup_monitor(std::string descriptive_name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, descriptive_name)) == m_monitors.end() ? NULL : *i;

		if(m != NULL)
		{
			log(Error) << "Monitor already in database." << endlog();
			return;
		}

		PropertyBag* p = new PropertyBag;
		m = new monitor;

		m->descriptive_name = descriptive_name;
		p->addProperty("descriptive_name", m->descriptive_name).doc("Descriptive name of the monitor");
		p->addProperty("active", m->active).doc("Is the monitor active?");
		p->addProperty("physical_part", m->part).doc("Robot part: ARM, BASE or BOTH");
		p->addProperty("control_space", m->space).doc("Compare in JOINT or CARTESIAN space");
		p->addProperty("physical_quantity", m->quantity).doc("Interesting quantity: POSITION, VELOCITY, FORCE or TORQUE");
		p->addProperty("event_type", m->e_type).doc("LEVEL or EDGE triggered event(s)");
		p->addProperty("compare_type", m->c_type).doc("LESS, LESS_EQUAL, EQUAL, GREATER, GREATER_EQUAL");
		p->addProperty("msg", m->msg).doc("Event's message");
		p->addProperty("epsilon", m->epsilon).doc("Epsilon range for the EQUAL compare_type (only).");
		p->addProperty("indices", m->indices).doc("Interesting states (indices).");
		p->addProperty("values", m->values).doc("Triggering state set-points");

		this->addProperty(m->descriptive_name, *p);
		m_monitors.push_back(m);
		m_active_monitors.reserve(m_monitors.size());
	}

	void YouBotMonitorService::activate_monitor(std::string name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) != m_monitors.end() ? *i : NULL;

		if(m == NULL)
		{
			log(Error) << "Monitor not found" << endlog();
			this->error();
			return;
		}

		if(m->active)
		{
			log(Warning) << "Cannot activate an already active monitor." << endlog();
			return;
		}

		if(m->indices.size() == 1)
		{
			m->is_single_value = true;
		}
		else
		{
			m->is_single_value = false;
		}

		if(m->space == CARTESIAN && (m->quantity == FORCE || m->quantity == TORQUE) )
		{
			log(Error) << "Cannot monitor cartesian FORCE or TORQUE at the moment." << endlog();
			this->error();
			return;
		}

		if(m->space == CARTESIAN)
		{
			m->id = control_space_toeventstring(m->space) + physical_quantity_toeventstring(m->quantity);
		}
		else
		{
			m->id.reserve(13);
			m->id = control_space_toeventstring(m->space);
			for(unsigned int i = 0; i < m->indices.size(); ++i)
			{
				m->id.append( boost::lexical_cast<string>(m->indices[i]));
			}
			m->id.append(physical_quantity_toeventstring(m->quantity));
		}


		if(!bind_function(m))
		{
			m->check = NULL;
			return;
		}

		m->active = true;
		m_active_monitors.push_back(m);
	}

	void YouBotMonitorService::deactivate_monitor(std::string name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) == m_monitors.end() ? NULL : *i;

		if(m == NULL)
		{
			log(Error) << "Monitor not found." << endlog();
			this->error();
			return;
		}

		m->active = false;

		m = (i = getMonitor(m_active_monitors, name)) == m_active_monitors.end() ? NULL : *i;
		if(m == NULL)
		{
			log(Warning) << "Monitor was not active." << endlog();
			return;
		}

		m_active_monitors.erase(i);
	}

	bool YouBotMonitorService::bind_function(monitor* m)
	{
		// Note: You MUST use the final (heap stored) monitor struct!

		if(m->space == CARTESIAN && m->part == BASE)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<nav_msgs::Odometry>), this, &m_base_cart_state, m->quantity, m->msg, &m->indices, &m->values, m->c_type, m->epsilon);
		}
		else if(m->space == CARTESIAN && m->part == ARM)
		{
//			m.check = boost::bind(&YouBotMonitorService::check_monitor, this, &arm_cart_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
			log(Error) << "Not implemented!" << endlog();
			return false;
		}
		else if(m->space == JOINT && m->part == BASE)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &m_base_joint_state, m->quantity, m->msg, &m->indices, &m->values, m->c_type, m->epsilon);
		}
		else if(m->space == JOINT && m->part == ARM)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &m_arm_joint_state, m->quantity, m->msg, &m->indices, &m->values, m->c_type, m->epsilon);
		}

		return true;
	}
}

ORO_CREATE_COMPONENT( YouBot::YouBotMonitorService )
