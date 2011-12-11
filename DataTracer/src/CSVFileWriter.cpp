#include "CSVFileWriter.hpp"

#include <iostream>

#include <ocl/Component.hpp>

using namespace RTT;

namespace DataTracer
{
  CSVFileWriter::CSVFileWriter(string const& name) :
      TaskContext(name), m_file_name("default.csv")
  {
    setupComponentInterface();
  }

  CSVFileWriter::~CSVFileWriter() {}

  void CSVFileWriter::setupComponentInterface()
  {
        this->addOperation("setFile",&CSVFileWriter::setFile,this);
        this->addOperation("writeLine",&CSVFileWriter::writeLine,this, OwnThread);

        this->addPort("buffer", buffer).doc("Connect to DataTracer");
  }

  bool CSVFileWriter::setFile(std::string filename)
  {
    if(this->getTaskState() == RTT::base::TaskCore::Stopped)
    {
      m_file_name = filename;
      return true;
    }
    return false;
  }

  bool CSVFileWriter::openFile()
  {
    // open file for writing
    try
    {
      m_file.open(m_file_name.c_str(), std::ios::out | std::ios::trunc);
    }
    catch(std::exception& e)
    {
      log(Error) << e.what();
      return false;
    }

    if(!m_file.is_open()) //open for output + delete previous contents
    {
      log(Error) << "Could not open file (" << m_file_name << ")." << endlog();
      return false;
    }

    return true;
  }

  bool CSVFileWriter::writeLine(std::string str)
  {
    if(!m_file.is_open() && !openFile())
    {
      return false;
    }

    m_file << str << endl;
    return true;
  }

  bool CSVFileWriter::startHook()
  {
    if(!m_file.is_open() && !openFile())
    {
      return false;
    }

    return TaskContext::startHook();
  }

  void CSVFileWriter::updateHook()
  {
    TaskContext::updateHook();

    RTT::FlowStatus flow = buffer.read(m_buffer);

    while(flow == NewData)
    {
      unsigned int size = m_buffer.data.size();
      m_file << m_buffer.data[0];
      for(unsigned int i = 1; i < size; ++i)
      {
        m_file << ", " << m_buffer.data[i];
      }
      m_file << endl;

      flow = buffer.read(m_buffer);
    }
  }

  void CSVFileWriter::stopHook()
  {
    m_file.flush();
    m_file.close();
    TaskContext::stopHook();
  }
}

ORO_CREATE_COMPONENT( DataTracer::CSVFileWriter )
