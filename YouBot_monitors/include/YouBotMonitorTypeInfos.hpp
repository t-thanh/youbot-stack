#pragma once

#include "YouBotMonitorHelpers.hpp"

#include <vector>
#include <iostream>

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/RTT.hpp>

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

	struct ControlSpaceTypeInfo : public RTT::types::TemplateTypeInfo<control_space, true>
	{
		ControlSpaceTypeInfo(): RTT::types::TemplateTypeInfo<control_space, true>( "control_space" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const control_space& in, RTT::PropertyBag& targetbag ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, control_space& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false; // unknown type !
		}
	};

	struct PhysicalPartTypeInfo : public RTT::types::TemplateTypeInfo<physical_part, true>
	{
		PhysicalPartTypeInfo(): RTT::types::TemplateTypeInfo<physical_part, true>( "physical_part" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_part& in, RTT::PropertyBag& targetbag ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_part& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false; // unknown type !
		}
	};

	struct PhysicalQuantityTypeInfo : public RTT::types::TemplateTypeInfo<physical_quantity, true>
	{
		PhysicalQuantityTypeInfo(): RTT::types::TemplateTypeInfo<physical_quantity, true>( "physical_quantity" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_quantity& in, RTT::PropertyBag& targetbag ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_quantity& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false; // unknown type !
		}
	};

	struct EventTypeTypeInfo : public RTT::types::TemplateTypeInfo<event_type, true>
	{
		EventTypeTypeInfo(): RTT::types::TemplateTypeInfo<event_type, true>( "event_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const event_type& in, RTT::PropertyBag& targetbag ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, event_type& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false; // unknown type !
		}
	};

	struct CompareTypeTypeInfo : public RTT::types::TemplateTypeInfo<compare_type, true>
	{
		CompareTypeTypeInfo(): RTT::types::TemplateTypeInfo<compare_type, true>( "compare_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const compare_type& in, RTT::PropertyBag& targetbag ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, compare_type& out ) const
		{
			log(Error) << "Not implemented!" << endlog();
			return false; // unknown type !
		}
	};

}
