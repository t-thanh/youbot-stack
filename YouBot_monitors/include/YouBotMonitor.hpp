#pragma once

#include <YouBot_monitors/typekit/Types.h>
#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <vector>

#include "YouBotMonitorHelpers.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;

    class YouBotMonitorService : public Service {

		public:
    		YouBotMonitorService(TaskContext* parent);
			virtual ~YouBotMonitorService();

			virtual void update();

			virtual void setup_monitor(std::string descriptive_name);

			virtual void activate_monitor(std::string name);

			virtual void deactivate_monitor(std::string name);

//			virtual void remove_monitors(std::string& name);

			virtual void setupComponentInterface();
			void emitEvent(std::string id, std::string message);
			void emitEvent(std::string id, std::string message, bool condition);

		protected:
			template<class message_type>
			bool check_monitor(InputPort<message_type>* const inp, const physical_quantity quantity, const std::string msg,
					vector<unsigned int>* const indices, vector<double>* const vector_value, compare_type c_type);

			bool bind_function(monitor* m);
			monitor* getMonitor(std::string& name);

			InputPort<sensor_msgs::JointState> 	base_joint_state;
			InputPort<nav_msgs::Odometry> 		base_cart_state;

			InputPort<sensor_msgs::JointState> 	arm_joint_state;
//			InputPort<> 						arm_cart_state;

			OutputPort<YouBot_monitors::monitor_event> events;
			YouBot_monitors::monitor_event m_events;

			vector<PropertyBag*> m_properties;
			vector<monitor*> m_monitors;

		public:
			PropertyBag sub_bag;
			std::string s_param;
			bool b_param;

			//active monitors -> lock free??
    };

	template<>
	bool YouBotMonitorService::check_monitor<sensor_msgs::JointState>(InputPort<sensor_msgs::JointState>* const imp, const physical_quantity quantity, const std::string msg,
			vector<unsigned int>* const indices, vector<double>* const cmp_value, compare_type c_type)
	{
		sensor_msgs::JointState tmp;
		if(imp->read(tmp) != NoData)
		{
			switch(quantity)
			{
				case(POSITION):
				{
					return compare(indices, cmp_value, tmp.position, c_type);
					break;
				}
				case(VELOCITY):
				{
					return compare(indices, cmp_value, tmp.velocity, c_type);
					break;
				}
				case(FORCE):
				{
					return compare(indices, cmp_value, tmp.effort, c_type);
					break;
				}
				case(TORQUE):
				{
					return compare(indices, cmp_value, tmp.effort, c_type);
					break;
				}
				default:
					log(Error) << "Case not recognized." << endlog();
					this->getOwner()->error();
					break;
			}
		}
		return false;
	}

	template<>
	bool YouBotMonitorService::check_monitor<nav_msgs::Odometry>(InputPort<nav_msgs::Odometry>* const imp, const physical_quantity quantity, const std::string msg,
			vector<unsigned int>* const indices, vector<double>* const vector_value, compare_type c_type)
	{
		nav_msgs::Odometry tmp;
		if(imp->read(tmp) != NoData)
		{
			switch(quantity)
			{
				case(POSITION):
				{
					vector<double> tmp2(3,0);
					tmp2[0] = tmp.pose.pose.position.x;
					tmp2[1] = tmp.pose.pose.position.y;
//					tmp2[2] = tmp.pose.pose.position.z;
					tmp2[2] = tf::getYaw (tmp.pose.pose.orientation); //yaw
					return compare(indices, vector_value, tmp2, c_type);
					break;
				}
				case(VELOCITY):
				{
					vector<double> tmp2(6,0);
					tmp2[0] = tmp.twist.twist.linear.x;
					tmp2[1] = tmp.twist.twist.linear.y;
					tmp2[2] = tmp.twist.twist.linear.z;
					tmp2[3] = tmp.twist.twist.angular.x;
					tmp2[4] = tmp.twist.twist.angular.y;
					tmp2[5] = tmp.twist.twist.angular.z;
					return compare(indices, vector_value, tmp2, c_type);
					break;
				}
				case(FORCE):
				{
					log(Error) << "FORCE not included in message." << endlog();
					this->getOwner()->error();
					break;
				}
				case(TORQUE):
				{
					log(Error) << "TORQUE not included in message." << endlog();
					this->getOwner()->error();
					break;
				}
				default:
				{
					log(Error) << "Case not recognized." << endlog();
					this->getOwner()->error();
					break;
				}
			}
		}
		return false;
	}

}
