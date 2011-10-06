#pragma once

//#include <vector>
#include <string>
#include <stdint.h>

namespace YouBot
{
	struct Poison
	{
		std::string originator; 	// Component/node id
		std::string description; 	// encoding still an issue
		float qos; 					// reliability of the channel //0..1 where 1 means healthy

		Poison() : originator(""), description(""), qos(0)
		{}

		Poison(const Poison& copy) : originator(copy.originator), description(copy.description), qos(copy.qos)
		{}
	};

	struct JointValue
	{
		uint64_t timeStamp; 	// Time of the data when value was created (microseconds since 1970)
		std::string joint_uri; 	// Joint id, where this message is about.
		std::string unit; 		// if empy expects si units, you can use boost::unit
		double value;

		JointValue() : timeStamp(0), joint_uri(""), unit(""), value(0)
		{}

		JointValue(const JointValue& copy) : timeStamp(copy.timeStamp), joint_uri(copy.joint_uri), unit(copy.unit), value(copy.value)
		{}
	};

	struct JointConstraint
	{
		std::string type; 		// smaller, greater, equal or <, >, =
		JointValue value;
		JointConstraint() : type(""), value()
		{}

		JointConstraint(const JointConstraint& copy) : type(copy.type), value(copy.value)
		{}
	};
}
