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
	const std::string E_OVERCURRENT = "e_OVERCURRENT";
	const std::string E_UNDERVOLTAGE = "e_UNDERVOLTAGE";
	const std::string E_OVERVOLTAGE = "e_OVERVOLTAGE";
	const std::string E_OVERTEMP = "e_OVERTEMP";
	const std::string E_EC_CONN_LOST = "e_EC_CONN_LOST";
	const std::string E_I2T_EXCEEDED = "e_I2T_EXCEEDED";
	const std::string E_HALL_ERR = "e_HALL_ERR";
	const std::string E_ENCODER_ERR = "e_ENCODER_ERR";
	const std::string E_SINE_COMM_INIT_ERR = "e_SINE_COMM_INIT_ERR";
	const std::string E_EMERGENCY_STOP = "e_EMERGENCY_STOP";
	const std::string E_EC_TIMEOUT = "e_EC_TIMEOUT";


	std::string ctrl_modes_tostring(const ctrl_modes& cd);

	std::string motor_status_tostring(const motor_status& cd);

	struct CtrlModesTypeInfo : public RTT::types::TemplateTypeInfo<ctrl_modes, true>
	{
		CtrlModesTypeInfo();

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const ctrl_modes& in, RTT::PropertyBag& targetbag ) const;

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, ctrl_modes& out ) const;
	};
}
