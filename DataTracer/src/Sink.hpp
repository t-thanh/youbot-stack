#pragma once

#include <rtt/RTT.hpp>
#include <rtt/ConnPolicy.hpp>

#include <YouBotTypes.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>

#include "DTTypes.hpp"

#include <vector>
#include <iostream>
#include <fstream>

using namespace RTT;

namespace DataTracer
{
  class Sink : public RTT::TaskContext
  {
    public:
      Sink(string const& name);
      ~Sink();

      bool addTrace(std::string foreign_port);

      void setupComponentInterface();

      bool startHook() ;
      void updateHook() ;

    private:
      base::PortInterface* find_foreign_port(std::string foreign_port);

      std::vector< boost::shared_ptr<ITraceTuple> > m_ports;
      OutputPort<std_msgs::Float64MultiArray> buffer;
      std_msgs::Float64MultiArray m_buffer;

      ros::Time m_start_time;
      RTT::ConnPolicy m_policy;
  };
}
