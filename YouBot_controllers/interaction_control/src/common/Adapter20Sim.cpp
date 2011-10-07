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

}


