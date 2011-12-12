#pragma once

#include <rtt/RTT.hpp>

#include <std_msgs/Float64MultiArray.h>

#include <vector>
#include <iostream>
#include <fstream>

using namespace RTT;

namespace DataTracer
{
  /**
   * @brief CSV FileWriter.
   */
  class CSVFileWriter : public RTT::TaskContext
  {
    public:
      CSVFileWriter(string const& name);
      ~CSVFileWriter();

      bool setFile(std::string filename);
      bool writeLine(std::string str);

      void setupComponentInterface();

      bool startHook() ;
      void updateHook() ;
      void stopHook();

    private:
      bool openFile();

      InputPort<std_msgs::Float64MultiArray> buffer;
      std_msgs::Float64MultiArray m_buffer;

      std::string m_file_name;
      std::ofstream m_file;
  };
}
