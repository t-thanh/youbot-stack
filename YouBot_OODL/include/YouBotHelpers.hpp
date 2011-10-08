#pragma once

#include "YouBotTypes.hpp"
#include <vector>
#include <iostream>

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/RTT.hpp>

namespace RTT
{
	namespace types
	{
		using namespace YouBot;

		std::ostream& operator<<(std::ostream& os, const ctrl_modes& cd);

		std::ostream& operator<<(std::ostream& os, const std::vector<ctrl_modes>& cd);

		std::istream& operator>>(std::istream& is, std::vector<ctrl_modes>& cd);

		std::istream& operator>>(std::istream& is, ctrl_modes& cd);

//		std::ostream& operator<<(std::ostream& os, const joint_status& cd);
//		std::istream& operator>>(std::istream& is, joint_status& cd);
	}
}

namespace YouBot
{
	std::string ctrl_modes_tostring(const ctrl_modes& cd);

	// joint_status is the superset of joint_error
	std::string joint_status_tostring(const joint_status& cd);

	struct CtrlModesTypeInfo : public RTT::types::TemplateTypeInfo<ctrl_modes, true>
	{
		CtrlModesTypeInfo();

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const ctrl_modes& in, RTT::PropertyBag& targetbag ) const;

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, ctrl_modes& out ) const;
	};
}
