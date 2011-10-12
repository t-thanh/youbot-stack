#pragma once

#include <boost/algorithm/string.hpp>
#include "vector"
#include "string"
#include "XVMatrix.h"
#include <std_msgs/Float64MultiArray.h>
#include <rtt/RTT.hpp>

namespace common20sim {

	using namespace RTT;

	typedef std_msgs::Float64MultiArray flat_matrix_t;
	typedef std_msgs::Float64MultiArray::_data_type flat_matrix_internal_t;

	/**
	 * @brief Removes illegal characters from 20sim generated names.
	 * This was necessary for interpretation based software, like the TaskBrowser, because they
	 * use things as '.' to access sub-properties.
	 */
	std::string replaceIllegalCharacter(std::string str);
	std::string makeShortName(std::string str);

	//TODO redefine =operators
	template<class T>
	class Adapter20Sim {

	private:
		T* m_port;
		std::string m_fullName;
		std::string m_shortName;
		std::string m_description;
		flat_matrix_t m_data;
		XVMatrix* m_link;

	public:
		Adapter20Sim(std::string name, std::string desc, XVMatrix* link, T* port) :
			m_port(port), m_description(desc), m_link(link)
		{
			m_fullName = replaceIllegalCharacter(name);
			m_shortName = makeShortName(name);

			// setup/resize m_data
			if(link != NULL)
			{
				m_data.data.resize(link->getColumns() * link->getRows(), 0);
			}
			else
			{
				log(Warning) << "XVMatrix unknown -> m_data size unknown." << endlog();
			}

			// setup the port
		}

		Adapter20Sim(const Adapter20Sim& copy)
		{
			m_port = copy.m_port;
			m_fullName = copy.m_fullName;
			m_shortName = copy.m_shortName;
			m_description = copy.m_description;
			m_data = copy.m_data;
			m_link = copy.m_link;
		}

		virtual ~Adapter20Sim()
		{
		}

		Adapter20Sim& operator=(const Adapter20Sim& ass)
		{
			log(Info) << "Assignment operator not implemented." << endlog();
			return *this;
		}

		std::string getFullName()
		{
			return m_fullName;
		}
		std::string getShortName()
		{
			return m_shortName;
		}
		std::string getDescription()
		{
			return m_description;
		}
		XVMatrix* getLink()
		{
			return m_link;
		}

		flat_matrix_t& getValue()
		{
			m_data.data.assign(m_link->getCArray().address(),m_link->getCArray().address()+m_link->getCArray().count());
			return m_data;
		}

		T* getPort()
		{
			return m_port;
		}

		void setValue(flat_matrix_t& rsh)
		{
			m_data.data.assign( rsh.data.begin(),rsh.data.end());
			m_link->setValues(m_data.data);
		}

	};
}

