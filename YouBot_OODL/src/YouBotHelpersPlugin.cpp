#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <iostream>
#include <YouBotTypes.h>
#include "YouBotHelpers.h"
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>
using namespace RTT;
using namespace std;

using namespace RTT;
using namespace RTT::types;
using namespace std;
/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    if ( t == 0 )
        cout << "Plugin of YouBot helpers loaded in process."<< endl;
    else
        cout << "Plugin of YouBot helpers in component: "<< t->getName() << endl;

	RTT::types::Types()->addType( new CtrlModesTypeInfo );
	RTT::types::Types()->addType( new SequenceTypeInfo<std::vector<ctrl_modes> >("std.vector<ctrl_modes>") );

//		RTT::types::Types()->addType( new JointStatusTypeInfo );
	RTT::types::Types()->addType( new SequenceTypeInfo<std::vector<joint_status> >("std.vector<joint_status>") );

	GlobalsRepository::shared_ptr globals = GlobalsRepository::Instance();
	globals->setValue( new Constant<ctrl_modes>("PLANE_ANGLE",PLANE_ANGLE) );
	globals->setValue( new Constant<ctrl_modes>("ANGULAR_VELOCITY",ANGULAR_VELOCITY) );
	globals->setValue( new Constant<ctrl_modes>("TORQUE",TORQUE) );
	globals->setValue( new Constant<ctrl_modes>("MOTOR_STOP",MOTOR_STOP) );
    return true;
}

std::string getRTTPluginName()
{
    return "YouBotHelpersPlugin";
}
