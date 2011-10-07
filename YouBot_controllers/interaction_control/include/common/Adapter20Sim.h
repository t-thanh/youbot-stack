#pragma once

#include <boost/algorithm/string.hpp>
#include "vector"
#include "string"
#include "XVMatrix.h"
#include <std_msgs/Float64MultiArray.h>

namespace common20sim {

	typedef std_msgs::Float64MultiArray flat_matrix_t;
	typedef std_msgs::Float64MultiArray::_data_type flat_matrix_internal_t;

	/**
	 * @brief Removes illegal characters from 20sim generated names.
	 * This was necessary for interpretation based software, like the TaskBrowser, because they
	 * use things as '.' to access sub-properties.
	 */
	std::string replaceIllegalCharacter(std::string str);

	//TODO redefine =operators
	template<class T>
	class Adapter20Sim {

	private:
		T* _port;
		std::string _fullName;
		std::string _shortName;
		std::string _description;
		flat_matrix_t _data;
		XVMatrix* _link;

	public:
		Adapter20Sim(std::string name, std::string desc, XVMatrix* link, T* port) :
			_port(port), _description(desc), _link(link)
		{
			_fullName = replaceIllegalCharacter(name);
			_shortName = makeShortName(name);

			// setup/resize _data
			// setup the port
		}

		Adapter20Sim(const Adapter20Sim& copy)
		{
			_port = copy._port;
			_fullName = copy._fullName;
			_shortName = copy._shortName;
			_description = copy._description;
			_data = copy._data;
			_link = copy._link;

		}

		virtual ~Adapter20Sim()
		{
		}

		std::string getFullName()
		{
			return _fullName;
		}
		std::string getShortName()
		{
			return _shortName;
		}
		std::string getDescription()
		{
			return _description;
		}
		XVMatrix* getLink()
		{
			return _link;
		}

		flat_matrix_t& getValue()
		{
			_data.data.assign(_link->getCArray().address(),_link->getCArray().address()+_link->getCArray().count());
			return _data;
		}

		T* getPort()
		{
			return _port;
		}

		void setValue(flat_matrix_t& rsh)
		{
			_data.data.assign( rsh.data.begin(),rsh.data.end());
			_link->setValues(_data.data);
		}

	private:
		std::string makeShortName(std::string str)
		{
			using namespace boost;
			int pos;
			pos=str.find_last_of("\\");
			if (pos!=std::string::npos)
			{
				std::string temp;
				temp=str.substr(pos+1,temp.length()-pos-1);
				return temp;
			}
			else
			{
				return str;
			}
		}

	};
}

