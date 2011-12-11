#pragma once

#include <rtt/RTT.hpp>
#include <rtt/InputPort.hpp>

#include <boost/tuple/tuple.hpp>
#include <vector>
#include <ostream>

#include <std_msgs/Float64.h>
#include <sensor_msgs/typekit/Types.h>

#include "DTFactory.hpp"

/**
 * Adding new type converters requires two steps:
 * 1) Implement: readAndConvert, configure and getElementNames
 * 2) Register the new type converter object with the Factory (generic design pattern)
 * For both steps see the examples in this file.
 */

namespace DataTracer
{

  // Float64MultiArray
  template<>
  RTT::FlowStatus DataTraceTuple<std_msgs::Float64MultiArray >::readAndConvert()
  {
    RTT::FlowStatus st = (*port).read(ptr);
    if(nr_of_elements != ptr->rvalue().data.size())
    {
      std::__throw_out_of_range(__N("DataTraceTuple<std_msgs::Float64MultiArray >"));
    }

    for(unsigned int i = 0; i < nr_of_elements; ++i)
    {
      buffer[i] = ptr->rvalue().data[i];
    }
    return st;
  }

  template<>
  bool DataTraceTuple<std_msgs::Float64MultiArray >::configure()
  {
    RTT::FlowStatus st = (*port).read(ptr);
    if(st == RTT::NoData)
      return false;
    nr_of_elements = ptr->rvalue().data.size();
    log(Debug) << port->getName() << " has nr_of_elements: " << nr_of_elements << endlog();
    return true;
  }

  template<>
  bool DataTraceTuple<std_msgs::Float64MultiArray >::getElementNames(std::stringstream& stream)
  {
    if(nr_of_elements == 0)
      return false;

    std::string base_name(port->getName());
    stream << base_name << "_0";
    for(unsigned int i = 1; i < nr_of_elements; ++i)
    {
      stream << ", " << base_name << "_" << i;
    }
    return true;
  }

  // Float64
  template<>
  RTT::FlowStatus DataTraceTuple<std_msgs::Float64 >::readAndConvert()
  {
    RTT::FlowStatus st = (*port).read(ptr);

    for(unsigned int i = 0; i < nr_of_elements; ++i)
    {
      buffer[i] = ptr->rvalue().data;
    }
    return st;
  }

  template<>
  bool DataTraceTuple<std_msgs::Float64 >::configure()
  {
    RTT::FlowStatus st = (*port).read(ptr);
    if(st == RTT::NoData)
      return false;
    nr_of_elements = 1;
    log(Debug) << port->getName() << " has nr_of_elements: " << nr_of_elements << endlog();
    return true;
  }

  template<>
  bool DataTraceTuple<std_msgs::Float64 >::getElementNames(std::stringstream& stream)
  {
    if(nr_of_elements == 0)
      return false;
    std::string base_name(port->getName());
    stream << base_name;
    return true;
  }

  // sensor_msgs::JointState
  template<>
  RTT::FlowStatus DataTraceTuple<sensor_msgs::JointState >::readAndConvert()
  {
    RTT::FlowStatus st = (*port).read(ptr);
    if(nr_of_elements != (ptr->rvalue().position.size() + ptr->rvalue().velocity.size() + ptr->rvalue().effort.size()) )
    {
      std::__throw_out_of_range(__N("DataTraceTuple<sensor_msgs::JointState >"));
    }

    int ct = 0;
    unsigned int tsize = ptr->rvalue().position.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      buffer[i] = ptr->rvalue().position[i];
    }
    ct = ptr->rvalue().position.size();

    tsize = ptr->rvalue().velocity.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      buffer[i+ct] = ptr->rvalue().velocity[i];
    }
    ct += ptr->rvalue().velocity.size();

    tsize = ptr->rvalue().effort.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      buffer[i+ct] = ptr->rvalue().effort[i];
    }

    return st;
  }

  template<>
  bool DataTraceTuple<sensor_msgs::JointState >::configure()
  {
    RTT::FlowStatus st = (*port).read(ptr);
    if(st == RTT::NoData)
      return false;
    nr_of_elements = ptr->rvalue().position.size() + ptr->rvalue().velocity.size() + ptr->rvalue().effort.size();
    log(Debug) << port->getName() << " has nr_of_elements: " << nr_of_elements << endlog();
    return true;
  }

  template<>
  bool DataTraceTuple<sensor_msgs::JointState >::getElementNames(std::stringstream& stream)
  {
    if(nr_of_elements == 0)
      return false;

    std::string base_name(port->getName());

    bool has_position = false;
    bool has_velocity = false;

    unsigned int tsize = ptr->rvalue().position.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      if(i == 0)
        stream << base_name << "_position_" << i;
      else
        stream << ", " << base_name << "_position_" << i;
      has_position = true;
    }

    tsize = ptr->rvalue().velocity.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      if(!has_position)
        stream << base_name << "_velocity_" << i;
      else
        stream << ", " << base_name << "_velocity_" << i;
      has_velocity = true;
    }

    tsize = ptr->rvalue().effort.size();
    for(unsigned int i = 0; i < tsize; ++i)
    {
      if(!has_velocity)
        stream << base_name << "_effort_" << i;
      else
        stream << ", " << base_name << "_effort_" << i;
    }
    return true;
  }

#define CREATE_RECORD(A,B) DTFactory::Instance()->Register(RTT::types::TypeInfoRepository::Instance()->getTypeById(&typeid(A))->getTypeName(), &B)

  namespace
  {
    ITraceTuple* CreateFloat64MultiArray()
    {
      return new DataTraceTuple<std_msgs::Float64MultiArray >;
    }

    ITraceTuple* CreateFloat64()
    {
      return new DataTraceTuple<std_msgs::Float64 >;
    }

    ITraceTuple* CreateJointState()
    {
      return new DataTraceTuple<sensor_msgs::JointState >;
    }

    static bool isRegistered1 = CREATE_RECORD(std_msgs::Float64MultiArray, CreateFloat64MultiArray);
    static bool isRegistered2 = CREATE_RECORD(std_msgs::Float64, CreateFloat64);
    static bool isRegistered3 = CREATE_RECORD(sensor_msgs::JointState, CreateJointState);
  }
}
