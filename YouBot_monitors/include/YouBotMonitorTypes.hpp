#pragma once

#include <boost/function.hpp>
#include <vector>

#include <ros/time.h>

#include <std_msgs/typekit/Types.h>

namespace YouBot
{
	typedef std_msgs::Float64MultiArray std_cart_t;
	typedef std_cart_t cart_efforts_t; // 6x1 vector (omega, trans)
	typedef std_cart_t homogeneous_matrix_t; // 4x4 matrix
	typedef std::vector<double> xyzypr_t; //internal

	const size_t max_event_length = 255;

	enum control_space 		{JOINT = 1, CARTESIAN = 2};
	enum physical_part 		{ARM = 1, BASE = 2}; //, BOTH = 3 - don't use BOTH yet
	enum physical_quantity 	{MONITOR_POSITION = 1, MONITOR_VELOCITY = 2, MONITOR_EFFORT = 3, MONITOR_TIME=4};
	enum event_type 		{EDGE = 1, LEVEL = 2};
	enum compare_type		{LESS = 1, LESS_EQUAL = 2, EQUAL = 3, GREATER = 4, GREATER_EQUAL = 5};

	typedef boost::function<bool() > monitor_fp;

	typedef struct _monitor
	{
		bool active;
		std::string descriptive_name;

		physical_part part;
		control_space space;
		physical_quantity quantity;
		event_type e_type;
		std::vector<compare_type> c_type;

		std::string id; // determined beforehand to speed up runtime execution.
		std::string msg;

		bool state; //for EDGE

		std::vector<bool> timer_state;
		std::vector<ros::Time> timer_expires;

		double epsilon;
		std::vector<unsigned int> indices;
		std::vector<double> values;

		monitor_fp check;

		_monitor() : active(false), part(ARM), space(JOINT), quantity(MONITOR_POSITION), e_type(EDGE), c_type(1, EQUAL),
				id(""), msg(""),
				state(false),
				timer_state(0),
				timer_expires(0),
				epsilon(5),
				indices(6),
				values(6)
		{}

		_monitor(const _monitor& copy) :
			active(false), part(copy.part), space(copy.space), quantity(copy.quantity), e_type(copy.e_type), c_type(copy.c_type),
				id(copy.id), msg(copy.msg),
				state(false),
				timer_state(0),
				timer_expires(0),
				epsilon(copy.epsilon),
				indices(copy.indices),
				values(copy.values)
		{}
	} monitor;

}
