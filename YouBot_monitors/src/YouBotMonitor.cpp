#include "YouBotMonitor.hpp"

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
		m_events.reserve(max_event_length);
		m_events.assign("");

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
        this->addOperation("copy_monitor",&YouBotMonitorService::copy_monitor,this, OwnThread);
        this->addOperation("assign_indices",&YouBotMonitorService::assign_indices,this, OwnThread);
        this->addOperation("assign_values",&YouBotMonitorService::assign_values,this, OwnThread);
        this->addOperation("activate_monitor",&YouBotMonitorService::activate_monitor,this, OwnThread);
        this->addOperation("deactivate_monitor",&YouBotMonitorService::deactivate_monitor,this, OwnThread);
        this->addOperation("remove_monitor",&YouBotMonitorService::remove_monitor,this, OwnThread);
        this->addOperation("listActiveMonitors",&YouBotMonitorService::listActiveMonitors,this, OwnThread);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message)
	{
		m_events = id + "." + message; //"jnt" + boost::lexical_cast<string>(joint)
		events.write(m_events);
	}

	void YouBotMonitorService::emitEvent(std::string id, std::string message, bool condition)
	{
		m_events = id + "." + message + "_" + (condition ? "true" : "false");
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

	void YouBotMonitorService::listActiveMonitors()
	{
		unsigned int size = m_active_monitors.size();
		for(unsigned int i = 0; i < size; ++i)
		{
			log(Info) << "[" << i << "] " << m_active_monitors[i]->descriptive_name << endlog();
		}

		if(size == 0)
		{
			log(Info) << "No active monitors." << endlog();
		}
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

	bool YouBotMonitorService::setup_monitor(std::string descriptive_name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, descriptive_name)) == m_monitors.end() ? NULL : *i;

		if(m != NULL)
		{
			log(Error) << "Monitor already in database." << endlog();
			return false;
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

		if(m_monitors.size() + 1 > m_monitors.capacity())
		{
			m_monitors.reserve(m_monitors.size() + 1);
			m_active_monitors.reserve(m_monitors.size() + 1);
		}

		m_monitors.push_back(m);

		return true;
	}

	bool YouBotMonitorService::copy_monitor(std::string source, std::string target)
	{
		vector<monitor*>::iterator i;
		monitor* m(NULL);
		monitor* m2(NULL);

		m = (i = getMonitor(m_monitors, source)) == m_monitors.end() ? NULL : *i;
		if(m == NULL)
		{
			log(Error) << "Monitor not in database." << endlog();
			return false;
		}

		m2 = (i = getMonitor(m_monitors, target)) == m_monitors.end() ? NULL : *i;
		if(m2 != NULL)
		{
			log(Error) << "Monitor already in database." << endlog();
			return false;
		}

		PropertyBag* p = new PropertyBag;
		m = new monitor(*m);

		m->descriptive_name = target;
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

		if(m_monitors.size() + 1 > m_monitors.capacity())
		{
			m_monitors.reserve(m_monitors.size() + 1);
			m_active_monitors.reserve(m_monitors.size() + 1);
		}

		m_monitors.push_back(m);

		return true;
	}

	bool YouBotMonitorService::assign_indices(std::string name, std::vector<uint32_t> indices)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) != m_monitors.end() ? *i : NULL;

		if(m == NULL)
		{
			log(Error) << "Monitor not found" << endlog();
			return false;
		}

		if(m->active)
		{
			log(Warning) << "Cannot assign indices to an active monitor." << endlog();
			return false;
		}

		m->indices = indices;
		return true;
	}

	bool YouBotMonitorService::assign_values(std::string name, std::vector<double> values)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) != m_monitors.end() ? *i : NULL;

		if(m == NULL)
		{
			log(Error) << "Monitor not found" << endlog();
			return false;
		}

		if(m->active)
		{
			log(Warning) << "Cannot assign values to an active monitor." << endlog();
			return false;
		}

		m->values  = values;
		return true;
	}

	bool YouBotMonitorService::activate_monitor(std::string name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) != m_monitors.end() ? *i : NULL;

		if(m == NULL)
		{
			log(Error) << "Monitor not found" << endlog();
			return false;
		}

		if(m->active)
		{
			log(Warning) << "Cannot activate an already active monitor." << endlog();
			return false;
		}

		if(m->indices.size() == 1)
		{
			m->is_single_value = true;
		}
		else
		{
			m->is_single_value = false;
		}

		if(m->quantity == MONITOR_TIME)
		{
			log(Debug) << "Ignoring physical_part, control_space, event_type, compare_type, epsilon and indices." << endlog();
			m->id = m->descriptive_name;
			m->e_type = LEVEL;
			unsigned int size = m->values.size();
			m->timer_state.reserve(size);
			m->timer_expires.reserve(size);
			ros::Time now = ros::Time::now();
			for(unsigned int i = 0; i < size; ++i)
			{
				m->timer_state[i] = false;

				int64_t sec_sum  = (int64_t)now.sec  + floor(m->values[i]);
				int64_t nsec_sum = (int64_t)now.nsec + (m->values[i] - floor(m->values[i])) * 1000000000ull;

				  // Throws an exception if we go out of 32-bit range
				ros::normalizeSecNSecUnsigned(sec_sum, nsec_sum);

				m->timer_expires[i] = ros::Time((uint32_t)sec_sum, (uint32_t)nsec_sum);
			}
		}
		else if(m->space == CARTESIAN && (m->quantity == MONITOR_FORCE || m->quantity == MONITOR_TORQUE) )
		{
			log(Error) << "Cannot monitor cartesian FORCE or TORQUE at the moment." << endlog();
			return false;
		}
		else if(m->space == CARTESIAN)
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
			return false;
		}

		m->active = true;
		m_active_monitors.push_back(m);

		return true;
	}

	bool YouBotMonitorService::deactivate_monitor(std::string name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) == m_monitors.end() ? NULL : *i;

		if(m == NULL)
		{
			log(Error) << "Monitor not found." << endlog();
			return false;
		}

		m->active = false;

		m = (i = getMonitor(m_active_monitors, name)) == m_active_monitors.end() ? NULL : *i;
		if(m == NULL)
		{
			log(Warning) << "Monitor was not active." << endlog();
			return false;
		}

		m_active_monitors.erase(i);

		return true;
	}

	bool YouBotMonitorService::remove_monitor(std::string name)
	{
		vector<monitor*>::iterator i;
		monitor* m = (i = getMonitor(m_monitors, name)) == m_monitors.end() ? NULL : *i;
		base::PropertyBase* pb = this->getProperty(name);

		if(m == NULL || pb == NULL)
		{
			log(Error) << "Monitor not found." << endlog();
			return false;
		}

		if(m->active)
		{
			log(Error) << "Cannot remove an active monitor." << endlog();
			return false;
		}

		m_monitors.erase(i);
		this->properties()->remove(pb);

		return true;
	}

	bool YouBotMonitorService::bind_function(monitor* m)
	{
		// Note: You MUST use the final (heap stored) monitor struct!

		if(m->quantity == MONITOR_TIME)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<ros::Time>), this, &(m->timer_expires[0]), m);
		}
		else if(m->space == CARTESIAN && m->part == BASE)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<nav_msgs::Odometry>), this, &m_base_cart_state, m);
		}
		else if(m->space == CARTESIAN && m->part == ARM)
		{
//			m.check = boost::bind(&YouBotMonitorService::check_monitor, this, &arm_cart_state, m.quantity, m.msg, &m.indices, &m.values, m.c_type);
			log(Error) << "Not implemented!" << endlog();
			return false;
		}
		else if(m->space == JOINT && m->part == BASE)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &m_base_joint_state, m);
		}
		else if(m->space == JOINT && m->part == ARM)
		{
			m->check = boost::bind(boost::mem_fn(&YouBotMonitorService::check_monitor<sensor_msgs::JointState>), this, &m_arm_joint_state, m);
		}

		return true;
	}
}

ORO_CREATE_COMPONENT( YouBot::YouBotMonitorService )
