#include "Adapter20Sim.h"

namespace common20sim {

	std::string replaceIllegalCharacter(std::string str)
	{
		using namespace boost;
		replace_all(str, "\\", "_");
		replace_all(str, "[", "__");
		replace_all(str, "]", "__");
		replace_all(str, ".", "_");
		replace_all(str, ",", "_");
		return str;
	}

	std::string makeShortName(std::string str)
	{
		size_t pos;
		pos=str.find_last_of("\\");
		if (pos != std::string::npos)
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

}


