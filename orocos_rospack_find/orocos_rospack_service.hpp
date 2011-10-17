#pragma once

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <ros/package.h>

using namespace RTT;
using namespace std;

/**
 * An example service which can be loaded in a component.
 */
class rospack : public RTT::Service {
public:
	rospack(TaskContext* owner);

    string find(string pack_name );
};
