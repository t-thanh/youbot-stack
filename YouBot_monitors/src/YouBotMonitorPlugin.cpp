#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <iostream>

#include "YouBotMonitorTypeInfos.hpp"

using namespace RTT;
using namespace std;

using namespace RTT;
using namespace RTT::types;
using namespace std;

using namespace YouBot;

std::string getRTTPluginName()
{
    return "YouBotMonitorPlugin";
}

/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    if ( t == 0 )
        cout << "Plugin of " << getRTTPluginName() << " loaded in process."<< endl;
    else
        cout << "Plugin of " << getRTTPluginName() << " loaded in component: "<< t->getName() << endl;

	RTT::types::Types()->addType( new ControlSpaceTypeInfo );
	RTT::types::Types()->addType( new PhysicalPartTypeInfo );
	RTT::types::Types()->addType( new PhysicalQuantityTypeInfo );
	RTT::types::Types()->addType( new EventTypeTypeInfo );
	RTT::types::Types()->addType( new CompareTypeTypeInfo );
//	RTT::types::Types()->addType( new SequenceTypeInfo<std::vector<unsigned int> >("std.vector<uint>") );

	GlobalsRepository::shared_ptr globals = GlobalsRepository::Instance();
	globals->setValue( new Constant<control_space>("JOINT",YouBot::JOINT) );
	globals->setValue( new Constant<control_space>("CARTESIAN",YouBot::CARTESIAN) );

	globals->setValue( new Constant<physical_part>("ARM",YouBot::ARM) );
	globals->setValue( new Constant<physical_part>("BASE",YouBot::BASE) );

	globals->setValue( new Constant<physical_quantity>("MONITOR_POSITION",YouBot::MONITOR_POSITION) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_VELOCITY",YouBot::MONITOR_VELOCITY) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_TORQUE",YouBot::MONITOR_TORQUE) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_FORCE",YouBot::MONITOR_FORCE) );

	globals->setValue( new Constant<event_type>("EDGE",YouBot::EDGE) );
	globals->setValue( new Constant<event_type>("LEVEL",YouBot::LEVEL) );

	globals->setValue( new Constant<compare_type>("LESS",YouBot::LESS) );
	globals->setValue( new Constant<compare_type>("LESS_EQUAL",YouBot::LESS_EQUAL) );
	globals->setValue( new Constant<compare_type>("EQUAL",YouBot::EQUAL) );
	globals->setValue( new Constant<compare_type>("GREATER",YouBot::GREATER) );
	globals->setValue( new Constant<compare_type>("GREATER_EQUAL",YouBot::GREATER_EQUAL) );

    return true;
}
