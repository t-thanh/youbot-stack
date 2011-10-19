#include "YouBotMonitorsTypeInfo.hpp"

namespace RTT
{
	namespace types
	{
		using namespace YouBot;


	}
}

namespace YouBot
{

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
