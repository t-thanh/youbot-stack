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

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	const size_t max_event_length = 255;

	enum control_space 		{JOINT = 1, CARTESIAN = 2};
	enum physical_part 		{ARM = 1, BASE = 2}; //, BOTH = 3 - don't use BOTH yet
	enum physical_quantity 	{POSITION = 1, VELOCITY = 2, TORQUE = 3, FORCE = 4};
	enum event_type 		{EDGE = 1, LEVEL = 2};
	enum compare_type		{LESS = 1, LESS_EQUAL = 2, GREATER = 3, GREATER_EQUAL = 4};

	typedef boost::function<bool() > monitor_fp;

	typedef struct _monitor
	{
		physical_part part;
		control_space space;
		physical_quantity quantity;
		event_type e_type;
		compare_type c_type;

		std::string id; // determined beforehand to speed up runtime execution.
		std::string msg;

		bool state; //for EDGE

		bool is_single_value;
		vector<unsigned int> indices;
		vector<double> values;

		monitor_fp check;

		_monitor() : part(ARM), space(JOINT), quantity(POSITION), e_type(EDGE), c_type(LESS),
				id(""), msg(""),
				state(false),
				is_single_value(true),
				indices(1),
				values(1)
		{}
	} monitor;

	bool compare(vector<unsigned int>* const indices, vector<double>* const first, const vector<double>& second, const compare_type ct)
	{
		unsigned int index = 0;
		for(unsigned int i = 0; i < indices->size(); ++i)
		{
			index = (*indices)[i];
			if( ct == LESS && (*first)[index] >= second[index] )
			{
				return false;
			}
			else if(ct == LESS_EQUAL && (*first)[index] > second[index])
			{
				return false;
			}
			else if(ct == GREATER && (*first)[index] <= second[index])
			{
				return false;
			}
			else if(ct == GREATER_EQUAL && (*first)[index] < second[index])
			{
				return false;
			}
		}
		return true;
	}

	std::string control_space_tostring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "cart";
		else if(space == JOINT)
			return "jnt";
		else
			return "error";
	}

	std::string physical_quantity_tostring(const physical_quantity& quantity)
	{
		if(quantity == POSITION)
			return "pos";
		else if(quantity == VELOCITY)
			return "vel";
		else if(quantity == FORCE)
			return "for";
		else if(quantity == TORQUE)
			return "tor";
		else
			return "error";
	}

// single value monitor vs state vector monitor

// Sensor msgs
// Channel name, <pos, velo, force>, limit_exceeded
// Channel name, <pos> pos_reached

// Odometry = Position + Twist
// pos, velo
// Howto: cartforce?

}

namespace RTT
{
	namespace types
	{
		using namespace YouBot;

		std::ostream& operator<<(std::ostream& os, const control_space& cd) {
			return os << control_space_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, control_space& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const physical_quantity& cd) {
			return os << physical_quantity_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, physical_quantity& cd) {
			return is >> cd;
		}

	}
}
