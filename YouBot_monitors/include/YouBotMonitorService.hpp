#pragma once

#include <YouBot_monitors/typekit/Types.h>
#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#include <boost/function.hpp>
#include <vector>

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	const size_t max_event_length = 255;

	enum control_space 		{JOINT = 1, CARTESIAN = 2};
	enum physical_part 		{ARM = 1, BASE = 2, BOTH = 3};
	enum physical_quantity 	{POSITION = 1, VELOCITY = 2, TORQUE = 3, FORCE = 4};
	enum event_type 		{LIMIT_EXCEEDED, REACHED};

	typedef boost::function<bool(double current_state) > single_value_fp;
	typedef boost::function<bool(vector<double> current_state) > state_vector_fp;
//	typedef boost::function<bool(double current_state)> check_fp;

	typedef struct _monitor
	{
		physical_part p;
		control_space ct;
		physical_quantity pt;
		event_type et;

		bool is_single_value;

		single_value_fp single_value;
		state_vector_fp state_vector;
	} monitor;

    class YouBotMonitorService : public Service {

		public:
    		YouBotMonitorService(const string& name, TaskContext* parent);
			virtual ~YouBotMonitorService();

			virtual void setup_monitor(physical_part& p, control_space& st, physical_quantity& pt, event_type& to_check, double single_value);
			virtual void setup_monitor(physical_part& p, control_space& st, physical_quantity& pt, event_type& to_check, std::vector<double> state_vector);

			virtual void remove_monitor(physical_part& p, control_space& st, physical_quantity& pt, event_type& to_check);
			virtual void clear_monitors();

			virtual void setupComponentInterface();
			void emitEvent(std::string id, std::string message);
			void emitEvent(std::string id, std::string message, bool condition);

		protected:
			InputPort<sensor_msgs::JointState> 	base_joint_state;
			InputPort<nav_msgs::Odometry> 		base_cart_state;

			InputPort<sensor_msgs::JointState> 	arm_joint_state;
//			InputPort<> 						arm_cart_state;

			OutputPort<YouBot_monitors::monitor_event> events;
			YouBot_monitors::monitor_event m_events;

			vector<monitor> m_monitors;
    };

// single value monitor vs state vector monitor

// Sensor msgs
// Channel name, <pos, velo, force>, limit_exceeded
// Channel name, <pos> pos_reached

// Odometry = Position + Twist
// pos, velo
// Howto: cartforce?

}
