#pragma once

#include <boost/function.hpp>
#include <vector>

namespace YouBot
{
	const size_t max_event_length = 255;

	enum control_space 		{JOINT = 1, CARTESIAN = 2};
	enum physical_part 		{ARM = 1, BASE = 2}; //, BOTH = 3 - don't use BOTH yet
	enum physical_quantity 	{POSITION = 1, VELOCITY = 2, TORQUE = 3, FORCE = 4};
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
		compare_type c_type;

		std::string id; // determined beforehand to speed up runtime execution.
		std::string msg;

		bool state; //for EDGE

		bool is_single_value;
		double epsilon;
		std::vector<unsigned int> indices;
		std::vector<double> values;

		monitor_fp check;

		_monitor() : active(false), part(ARM), space(JOINT), quantity(POSITION), e_type(EDGE), c_type(LESS),
				id(""), msg(""),
				state(false),
				is_single_value(true),
				epsilon(5),
				indices(6),
				values(6)
		{}
	} monitor;

}
