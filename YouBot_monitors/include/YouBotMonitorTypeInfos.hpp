#pragma once

#include "YouBotMonitorHelpers.hpp"

#include <vector>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

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

		std::ostream& operator<<(std::ostream& os, const physical_part& cd) {
			return os << physical_part_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, physical_part& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const event_type& cd) {
			return os << event_type_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, event_type& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const compare_type& cd) {
			return os << compare_type_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, compare_type& cd) {
			return is >> cd;
		}

	}
}

namespace YouBot
{
	using namespace RTT;

	struct ControlSpaceTypeInfo : public RTT::types::TemplateTypeInfo<control_space, true>
	{
		ControlSpaceTypeInfo(): RTT::types::TemplateTypeInfo<control_space, true>( "control_space" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const control_space& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("control_space");
			targetbag.add( new Property<std::string>("control_space", "", control_space_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, control_space& out ) const
		{
			if ( bag.getType() == std::string("control_space") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("control_space"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "JOINT"))
				{
					out = JOINT;
				}
				else if(boost::equals(str, "CARTESIAN"))
				{
					out = CARTESIAN;
				}
				else
				{
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

	struct PhysicalPartTypeInfo : public RTT::types::TemplateTypeInfo<physical_part, true>
	{
		PhysicalPartTypeInfo(): RTT::types::TemplateTypeInfo<physical_part, true>( "physical_part" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_part& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("physical_part");
			targetbag.add( new Property<std::string>("physical_part", "", physical_part_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_part& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return true; // unknown type !
		}
	};

	struct PhysicalQuantityTypeInfo : public RTT::types::TemplateTypeInfo<physical_quantity, true>
	{
		PhysicalQuantityTypeInfo(): RTT::types::TemplateTypeInfo<physical_quantity, true>( "physical_quantity" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_quantity& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("physical_quantity");
			targetbag.add( new Property<std::string>("physical_quantity", "", physical_quantity_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_quantity& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return true; // unknown type !
		}
	};

	struct EventTypeTypeInfo : public RTT::types::TemplateTypeInfo<event_type, true>
	{
		EventTypeTypeInfo(): RTT::types::TemplateTypeInfo<event_type, true>( "event_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const event_type& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("event_type");
			targetbag.add( new Property<std::string>("event_type", "", event_type_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, event_type& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return true; // unknown type !
		}
	};

	struct CompareTypeTypeInfo : public RTT::types::TemplateTypeInfo<compare_type, true>
	{
		CompareTypeTypeInfo(): RTT::types::TemplateTypeInfo<compare_type, true>( "compare_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const compare_type& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("compare_type");
			targetbag.add( new Property<std::string>("compare_type", "", compare_type_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, compare_type& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return true; // unknown type !
		}
	};

}
