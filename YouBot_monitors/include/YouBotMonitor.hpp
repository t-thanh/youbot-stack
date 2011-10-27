#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <vector>

#include "YouBotMonitorHelpers.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;

    class YouBotMonitorService : public TaskContext {

		public:
    		YouBotMonitorService(const string& name);
			virtual ~YouBotMonitorService();

			virtual bool startHook();

			virtual void updateHook();

			virtual void listActiveMonitors();

			virtual bool setup_monitor(std::string descriptive_name);

			virtual bool activate_monitor(std::string name);

			/**
			 * @brief Lua cannot access PropertyBags, this is the work around.
			 */
			virtual bool assign_indices(std::string, std::vector<uint32_t> indices);

			/**
			 * @brief Lua cannot access PropertyBags, this is the work around.
			 */
			virtual bool assign_values(std::string, std::vector<double> values);

			virtual bool copy_monitor(std::string source, std::string target);

			virtual bool deactivate_monitor(std::string name);

			virtual bool remove_monitor(std::string name);

			virtual void setupComponentInterface();
			void emitEvent(std::string id, std::string message);
			void emitEvent(std::string id, std::string message, bool condition);

		protected:
			template<class message_type>
			bool check_monitor(message_type* const inp, monitor* const mon);

			bool bind_function(monitor* m);
			vector<monitor*>::iterator getMonitor(vector<monitor*>& list, std::string& name);

			InputPort<sensor_msgs::JointState> 	base_joint_state;
			InputPort<nav_msgs::Odometry> 		base_cart_state;

			InputPort<sensor_msgs::JointState> 	arm_joint_state;
//			InputPort<> 						arm_cart_state;

			sensor_msgs::JointState m_base_joint_state;
			nav_msgs::Odometry m_base_cart_state;
			sensor_msgs::JointState m_arm_joint_state;
//			m_arm_cart_state

			OutputPort<std::string> events;
			std::string m_events;

			vector<PropertyBag*> m_properties;
			vector<monitor*> m_monitors;

			vector<monitor*> m_active_monitors; //TODO: Change into lockfree list?
    };

	template<>
	bool YouBotMonitorService::check_monitor<sensor_msgs::JointState>(sensor_msgs::JointState* const imp, monitor* const mon)
	{
		switch(mon->quantity)
		{
			case(MONITOR_POSITION):
			{
				return compare(mon->indices, mon->values, imp->position, mon->c_type, mon->epsilon);
				break;
			}
			case(MONITOR_VELOCITY):
			{
				return compare(mon->indices, mon->values, imp->velocity, mon->c_type, mon->epsilon);
				break;
			}
			case(MONITOR_FORCE):
			{
				return compare(mon->indices, mon->values, imp->effort, mon->c_type, mon->epsilon);
				break;
			}
			case(MONITOR_TORQUE):
			{
				return compare(mon->indices, mon->values, imp->effort, mon->c_type, mon->epsilon);
				break;
			}
			default:
			{
				log(Error) << "Case not recognized." << endlog();
				this->error();
				break;
			}
		}
		return false;
	}

	template<>
	bool YouBotMonitorService::check_monitor<nav_msgs::Odometry>(nav_msgs::Odometry* const imp, monitor* const mon)
	{
		switch(mon->quantity)
		{
			case(MONITOR_POSITION):
			{
				vector<double> tmp2(3,0);
				tmp2[0] = imp->pose.pose.position.x;
				tmp2[1] = imp->pose.pose.position.y;
//					tmp2[2] = tmp.pose.pose.position.z;
				tmp2[2] = tf::getYaw (imp->pose.pose.orientation); //yaw
				return compare(mon->indices, mon->values, tmp2, mon->c_type, mon->epsilon);
				break;
			}
			case(MONITOR_VELOCITY):
			{
				vector<double> tmp2(6,0);
				tmp2[0] = imp->twist.twist.linear.x;
				tmp2[1] = imp->twist.twist.linear.y;
				tmp2[2] = imp->twist.twist.linear.z;
				tmp2[3] = imp->twist.twist.angular.x;
				tmp2[4] = imp->twist.twist.angular.y;
				tmp2[5] = imp->twist.twist.angular.z;
				return compare(mon->indices, mon->values, tmp2, mon->c_type, mon->epsilon);
				break;
			}
			case(MONITOR_FORCE):
			{
				log(Error) << "FORCE not included in message." << endlog();
				this->error();
				break;
			}
			case(MONITOR_TORQUE):
			{
				log(Error) << "TORQUE not included in message." << endlog();
				this->error();
				break;
			}
			default:
			{
				log(Error) << "Case not recognized." << endlog();
				this->error();
				break;
			}
		}

		return false;
	}

	template<>
	bool YouBotMonitorService::check_monitor<ros::Time>(ros::Time* const imp, monitor* const mon)
	{
		unsigned int size = mon->indices.size();
		ros::Time now = ros::Time::now();

		for(unsigned int i = 0; i < size; ++i)
		{
			if(!mon->timer_state[i] && now > mon->timer_expires[i])
			{
				mon->timer_state[i] = true;
				return true;
			}
		}
		return false;
	}

}
