#pragma once

#include <string>
#include <vector>

#include "YouBot_primitives.hpp"

namespace YouBot
{
	struct JointValues
	{
		Poison poisonStamp;
		std::string model_uri; // the model which the message conforms to
		std::vector<JointValue> values;

		JointValues() : poisonStamp(), model_uri(""), values(0, JointValue())
		{}

		JointValues(int number_of_values) : poisonStamp(), model_uri(""), values(number_of_values, JointValue())
		{}

		JointValues(const JointValues& copy) : poisonStamp(copy.poisonStamp), model_uri(copy.model_uri), values(copy.values)
		{}
	};

	struct JointConstraints
	{
		std::string model_uri; // the model which the message conforms to
		std::vector<JointConstraint> values;

		JointConstraints() : model_uri(""), values(0, JointConstraint())
		{}

		JointConstraints(int number_of_values) : model_uri(""), values(number_of_values, JointConstraint())
		{}

		JointConstraints(const JointConstraints& copy) : model_uri(copy.model_uri), values(copy.values)
		{}
	};
}

