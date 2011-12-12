#pragma once

#include <rtt/RTT.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/base/DataSourceBase.hpp>
#include <rtt/internal/DataSource.hpp>

#include <vector>
#include <string>

namespace DataTracer
{
  /**
   * @brief Abstract data structure for converting any message type into doubles for CSV storage.
   */
  struct ITraceTuple
  {
    ITraceTuple() : nr_of_elements(0), buffer(NULL) {}

    virtual ~ITraceTuple() {}

    /**
     * @brief The name given to the antiCloned port.
     */
    boost::shared_ptr<std::string> port_name;

    /**
     * @brief The antiClone() port.
     */
    boost::shared_ptr<RTT::base::InputPortInterface> port;

    /**
     * @brief The 'fixed' number of variables copied from the message to the buffer.
     */
    unsigned int nr_of_elements;

    /**
     * @brief Intermediate storage buffer.
     */
    std::vector<double>::iterator buffer;

    /**
     * @brief Reads the InputPort<T> and converts the (selected) message elements into a vector<double>
     */
    virtual RTT::FlowStatus readAndConvert() = 0;

    /**
     * @brief CSV names.
     */
    virtual bool getElementNames(std::stringstream& stream) = 0;

    /**
     * @brief Read the data sample and determine the number of elements.
     * @note The OutputPort<T> MUST have set a data sample for this to work proper.
     */
    virtual bool configure() = 0;
  };

  /**
   * @brief Implementation for specific ITracTuple types.
   */
  template<class DataType>
  struct DataTraceTuple : ITraceTuple
  {
    DataTraceTuple()
    {
      ptr.reset(new RTT::internal::ValueDataSource<DataType>);
    }

    virtual ~DataTraceTuple() { };

    // Implement:
    RTT::FlowStatus readAndConvert();
    bool configure();
    bool getElementNames(std::stringstream& stream);

    /**
     * @brief DataSourceBase for InputPort<T>.read() operation.
     */
    typename RTT::internal::ValueDataSource<DataType>::shared_ptr ptr;
  };

}
