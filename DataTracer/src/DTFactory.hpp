#pragma once

#include "ITraceTuple.hpp"

#include <rtt/RTT.hpp>

using namespace RTT;

namespace DataTracer
{

  class DTFactory;

  namespace {
    boost::shared_ptr<DTFactory> dtfactory;
  }

  typedef boost::function<ITraceTuple* (void)> ProductCreator;
  typedef std::string IdentifierType;
  typedef ITraceTuple AbstractProduct;

  // Copied from LUNA
  // template <class AbstractProduct, typename IdentifierType, typename ProductCreator = AbstractProduct* (*)(), template<typename, class> class FactoryErrorPolicy = DefaultFactoryError>
  class DTFactory
  {
    public:
      bool Register(const IdentifierType& id, ProductCreator creator)
      {
        // LOG(LUNA::LOG_TRACE, "GenericFactory::Register\n");
        return m_mapping.insert(std::pair<IdentifierType, ProductCreator>(id, creator)).second; //returns an std::pair<iterator,bool>
      }

      bool Unregister(const IdentifierType& id)
      {
        return m_mapping.erase(id) == 1; //1, if element erased
      }

      AbstractProduct* CreateObject(const IdentifierType& id)
      {
        MapType::iterator i = m_mapping.find(id);
        if (i != m_mapping.end())
        {
          return (i->second)();
        }
        else
        {
          log(Error) << "GenericFactory::CreateObject - identifier type not known.\n" << endlog();
          return NULL; //not found
        }
      }

      static boost::shared_ptr<DTFactory> Instance()
      {
          if ( dtfactory )
              return dtfactory;
          dtfactory.reset( new DTFactory() );

          return dtfactory;
      }

      ~DTFactory()
      {
        log(Debug) << "~DTFactory() executed." << endlog();
      }

      DTFactory()
      { }

    private:
      typedef std::map<IdentifierType, ProductCreator> MapType;
      MapType m_mapping;
  };
}
