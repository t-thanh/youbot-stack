#include "Sink.hpp"

#include <iostream>

#include <ocl/Component.hpp>
#include <rtt/Service.hpp>

#include "boost/tuple/tuple.hpp"
#include <boost/algorithm/string.hpp>

#include "DTFactory.hpp"

using namespace RTT;

namespace DataTracer
{
  Sink::Sink(string const& name) :
      TaskContext(name), m_ports()
  {
    m_policy.type = ConnPolicy::DATA;
    m_policy.init = true;
    m_policy.lock_policy = ConnPolicy::LOCK_FREE;

    setupComponentInterface();
  }

  Sink::~Sink() {}

  void Sink::setupComponentInterface()
  {
        this->addOperation("addTrace",&Sink::addTrace,this);

        this->addPort("buffer", buffer).doc("Connect to FileWriter");
  }

  base::PortInterface* Sink::find_foreign_port(std::string foreign_port)
  {
    // Check if the port exists
    base::PortInterface* port(NULL);
    std::vector<std::string> address;
    boost::split(address, foreign_port, boost::is_any_of("."));
    if(address.size() != 2 && address.size() != 3)
    {
      log(Error) << "Cannot use the supplied string to find the foreign_port: "<< foreign_port << endlog();
      log(Info) << "Use: component.port or component.service.port" << endlog();
      return false;
    }

    TaskContext* task_ptr(getPeer(address[0]));
    if(task_ptr == NULL)
    {
      log(Error) << "Cannot find the peer component (" << address[0] << ")." << endlog();
      return NULL;
    }

    if(address.size() == 2)
    {
      port = task_ptr->getPort(address[1]);
    }
    else if(address.size() == 3)
    {
      Service::shared_ptr service( task_ptr->provides(address[1]) );
      if(service == NULL)
      {
        log(Error) << "Cannot find the service (" << address[1] << ") in peer component (" << address[0] << ")." << endlog();
        return NULL;
      }

      port = service->getPort(address[2]);
    }

    if(port == NULL)
    {
      if(address.size () == 2)
        log(Error) << "Cannot find port (" << address[1] << ") in peer component (" << address[0] << ")." << endlog();
      else
        log(Error) << "Cannot find port (" << address[2] << ") in service (" << address[1] << ") of peer component (" << address[0] << ")." << endlog();
      return NULL;
    }

    return port;
  }

  bool Sink::addTrace(std::string foreign_port)
  {
    if(this->getTaskState() == RTT::base::TaskCore::Stopped)
    {
      // check if it is not already traced
      for(unsigned int i = 0; i < m_ports.size(); ++i)
      {
        if( !foreign_port.compare( *(m_ports[i]->port_name) ) )
        {
          log(Error) << "Port (" << foreign_port << ") already traced." << endlog();
          return false;
        }
      }

      // Find the output port
      base::PortInterface* port(NULL);
      if( (port = find_foreign_port(foreign_port)) == NULL)
      {
        return false;
      }

      // check if it is an output port
      base::OutputPortInterface* outp_port(NULL);
      outp_port = dynamic_cast<base::OutputPortInterface*>(port);
      if(outp_port == NULL)
      {
        log(Error) << "Cannot trace an input port, please use the corresponding output port." << endlog();
        return false;
      }

      // create antiClone -> InputPortInterface
      base::PortInterface* tptr(outp_port->antiClone());

      base::InputPortInterface* inp_port(dynamic_cast<base::InputPortInterface*>(tptr));
      if(inp_port == NULL)
      {
        log(Error) << "Could not convert the antiClone" << endlog(); // this should not happen
        return false;
      }

      // Change all '.' into '_' -> generates 'unique' ports
      std::string tmp(foreign_port);
      boost::replace_all(tmp, ".", "_");
      std::string* port_name_ptr(new std::string(tmp));// create permanent string

      // Set name to port_name
      inp_port->setName(*port_name_ptr);

      // create variable for intermediate storage
      ITraceTuple* tuple = DTFactory::Instance()->CreateObject(inp_port->getTypeInfo()->getTypeName());

      if(tuple == NULL)
      {
        log(Info) << "DTFactory failed, type unknown." << endlog();
        return false;
      }

      // add port to interface
      std::stringstream doc;
      this->addPort(*port_name_ptr, *inp_port).doc(doc.str());

      // Create stream
      outp_port->connectTo(inp_port, m_policy); // connection policy

      // Store in port database
      tuple->port_name.reset(port_name_ptr);
      tuple->port.reset(inp_port);
      m_ports.push_back(boost::shared_ptr<ITraceTuple>(tuple));

      log(Info) << "Added trace: " << *port_name_ptr << endlog();
      return true;
    }
    else
    {
      log(Info) << "Can only add trace in STOPPED state." << endlog();
      return false;
    }
  }

  bool Sink::startHook()
  {
    unsigned int buffer_space(1); // 0 == base time (backup, for other messages that do not have time stamps)
    std::stringstream column_buffer;
    column_buffer << "time, ";

    for(unsigned int i = 0; i < m_ports.size(); ++i)
    {
      if(!m_ports[i]->configure())// Determine amongst others the number of data elements of each port
      {
        log(Error) << "Could not configure port: " << *(m_ports[i]->port_name) << endlog();
        return false;
      }
      buffer_space += m_ports[i]->nr_of_elements;

      if(i != 0)
      {
        column_buffer << ", ";
      }

      if(!m_ports[i]->getElementNames(column_buffer))
      {
        log(Error) << "Could not get the column names for port: " << *(m_ports[i]->port_name) << endlog();
        return false;
      }
    }

    // Create the write buffer
    m_buffer.data.resize(buffer_space, 0.0);
    buffer.setDataSample(m_buffer);

    // Setup buffer refs
    unsigned int elems = 1;
    for(unsigned int i = 0; i < m_ports.size(); ++i)
    {
      m_ports[i]->buffer = m_buffer.data.begin()+elems;
      elems += m_ports[i]->nr_of_elements;
    }

    // store column names in file writer
    OperationCaller<void(std::string) > op_setColumnNames;
    TaskContext* task_ptr = getPeer("FileWriter");
    if(task_ptr == NULL)
    {
      log(Error) << "Could not find peer FileWriter" << endlog();
      return false;
    }
    op_setColumnNames = task_ptr->getOperation("writeLine");

    if(!op_setColumnNames.ready())
    {
      log(Error) << "Could not connect to FileWriter.writeLine" << endlog();
      return false;
    }
    op_setColumnNames(column_buffer.str());

    m_start_time = ros::Time::now();

    return TaskContext::startHook();
  }

  void Sink::updateHook()
  {
    TaskContext::updateHook();

    m_buffer.data[0] = (ros::Time::now() - m_start_time).toSec();

    for(unsigned int i = 0; i < m_ports.size(); ++i)
    {
      m_ports[i]->readAndConvert();
    }

    buffer.write(m_buffer);
  }
}

ORO_CREATE_COMPONENT( DataTracer::Sink )
