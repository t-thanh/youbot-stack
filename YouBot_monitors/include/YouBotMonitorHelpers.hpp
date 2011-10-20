#pragma once

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
		vector<unsigned int> indices;
		vector<double> values;

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

	bool compare(vector<unsigned int>* const indices, vector<double>* const first, const vector<double>& second, const compare_type ct, const double epsilon)
	{
		unsigned int index = 0;
		unsigned int size = indices->size();
		for(unsigned int i = 0; i < size; ++i)
		{
			index = (*indices)[i];
			if( ct == LESS && abs((*first)[index] - second[index]) >= epsilon )
			{
				return false;
			}
			else if(ct == LESS_EQUAL && abs((*first)[index] - second[index]) > epsilon)
			{
				return false;
			}
			else if(ct == GREATER && abs((*first)[index] - second[index]) <= epsilon)
			{
				return false;
			}
			else if(ct == GREATER_EQUAL && abs((*first)[index] - second[index]) < epsilon)
			{
				return false;
			}
		}

		return (size > 0) ? true : false;
	}

	std::string control_space_tostring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "CARTESIAN";
		else if(space == JOINT)
			return "JOINT";
		else
			return "error";
	}

	std::string control_space_toeventstring(const control_space& space)
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
			return "POSITION";
		else if(quantity == VELOCITY)
			return "VELOCITY";
		else if(quantity == FORCE)
			return "FORCE";
		else if(quantity == TORQUE)
			return "TORQUE";
		else
			return "error";
	}

	std::string physical_quantity_toeventstring(const physical_quantity& quantity)
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

	std::string physical_part_tostring(const physical_part& part)
	{
		if(part == ARM)
			return "ARM";
		else if(part == BASE)
			return "BASE";
		else
			return "error";
	}

	std::string event_type_tostring(const event_type& e_type)
	{
		if(e_type == EDGE)
			return "EDGE";
		else if(e_type == LEVEL)
			return "LEVEL";
		else
			return "error";
	}

	std::string compare_type_tostring(const compare_type& c_type)
	{
		if(c_type == LESS)
			return "LESS";
		else if(c_type == LESS_EQUAL)
			return "LESS_EQUAL";
		else if(c_type == GREATER)
			return "GREATER";
		else if(c_type == GREATER_EQUAL)
			return "GREATER_EQUAL";
		else
			return "error";
	}

}
