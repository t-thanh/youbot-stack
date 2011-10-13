#include "YouBotHelpers.hpp"

#include <rtt/Logger.hpp>
#include <ostream>
#include <istream>
#include <boost/algorithm/string.hpp>
#include <youbot/ProtocolDefinitions.hpp>

namespace RTT
{
	namespace types
	{
		using namespace YouBot;
		using namespace RTT;

		std::ostream& operator<<(std::ostream& os, const ctrl_modes& cd) {
//			log(Info) << "O operator<< ctrl_modes" << endlog();
			return os << ctrl_modes_tostring(cd);
		}

		std::ostream& operator<<(std::ostream& os, const std::vector<ctrl_modes>& cd) {
//			log(Info) << "O operator<< vector<ctrl_modes>" << endlog();
			for(unsigned int i = 0; i < cd.size(); ++i)
			{
				os << cd[i];
			}
			return os;
		}

		std::istream& operator>>(std::istream& is, std::vector<ctrl_modes>& cd) {
//			log(Info) <<  "I operator>> vector<ctrl_modes>" << endlog();
			char c;
			for(unsigned int i = 0; i < cd.size(); ++i)
			{
				is >> c >> cd[i]; // '{' number || ',' number
			}
			return is >> c; // '}'
		}

		std::istream& operator>>(std::istream& is, ctrl_modes& cd) {
//			log(Info) <<  "I operator>> ctrl_modes" << endlog();
			return is >> cd;
		}
	}
}

namespace YouBot
{

	using namespace RTT;

	std::string ctrl_modes_tostring(const ctrl_modes& cd)
	{
		switch(cd)
		{
			case(PLANE_ANGLE):
				return "PLANE_ANGLE";
				break;
			case(ANGULAR_VELOCITY):
				return "ANGULAR_VELOCITY";
				break;
			case(TORQUE):
				return "TORQUE";
				break;
			case(MOTOR_STOP):
				return "MOTOR_STOP";
				break;
			default:
				log(Error) << "Control mode not recognized." << endlog();
				return "UNKNOWN";
		}
	}

	std::string motor_status_tostring(const motor_status& cd)
	{
		std::stringstream errors;

		if (cd & OVER_CURRENT) {
		  errors << "OVER_CURRENT ";
		}

		if (cd & UNDER_VOLTAGE) {
		  errors << "UNDER_VOLTAGE ";
		}

		if (cd & OVER_VOLTAGE) {
		  errors << "OVER_VOLTAGE ";
		}

		if (cd & OVER_TEMPERATURE) {
		  errors << "OVER_TEMPERATURE ";
		}

		if (cd & MOTOR_HALTED) {
		  errors << "MOTOR_HALTED ";
		}

		if (cd & HALL_SENSOR_ERROR) {
		  errors << "HALL_SENSOR_ERROR ";
		}

	//    if (messageBuffer.stctInput.errorFlags & ENCODER_ERROR) {
	//      statusMessages.push_back(errorMessage + "got encoder problem");
	//    }
	//
	//     if (messageBuffer.stctInput.errorFlags & INITIALIZATION_ERROR) {
	//      statusMessages.push_back(errorMessage + "got inizialization problem");
	//    }

		if (cd & PWM_MODE_ACTIVE) {
		  errors << "PWM_MODE_ACTIVE ";
		}

		if (cd & VELOCITY_MODE) {
		  errors << "VELOCITY_MODE ";
		}

		if (cd & POSITION_MODE) {
		  errors << "POSITION_MODE ";
		}

		if (cd & TORQUE_MODE) {
		  errors << "TORQUE_MODE ";
		}

	//    if (messageBuffer.stctInput.errorFlags & EMERGENCY_STOP) {
	//      statusMessages.push_back(errorMessage + "has emergency stop active");
	//    }
	//
	//    if (messageBuffer.stctInput.errorFlags & FREERUNNING) {
	//      statusMessages.push_back(errorMessage + "has freerunning active");
	//    }

		if (cd & POSITION_REACHED) {
		  errors << "POSITION_REACHED ";
		}

		if (cd & INITIALIZED) {
		  errors << "INITIALIZED ";
		}

		if (cd & TIMEOUT) {
		  errors << "TIMEOUT ";
		}

		if (cd & I2T_EXCEEDED) {
		  errors << "I2T_EXCEEDED ";
		}

		if(errors.str() == "")
		{
			errors << "OK";
		}

		return errors.str();
	}

	CtrlModesTypeInfo::CtrlModesTypeInfo() : RTT::types::TemplateTypeInfo<ctrl_modes, true>( "ctrl_modes" )
	{ }

	// this is a helper function, which is called by composeType() of the same class:
	bool CtrlModesTypeInfo::decomposeTypeImpl(const ctrl_modes& in, RTT::PropertyBag& targetbag ) const {
		log(Error) << "Not implemented!" << endlog();
		return false;
	}

	bool CtrlModesTypeInfo::composeTypeImpl(const RTT::PropertyBag& bag, ctrl_modes& out ) const
	{
		log(Error) << "Not implemented!" << endlog();
		return false; // unknown type !
	}

}
