#include "orocos_rospack_service.hpp"
using namespace RTT;
using namespace std;

/**
 * An example service which can be loaded in a component.
 */
rospack::rospack(TaskContext* owner) :
		Service("rospack", owner)
{
	this->addOperation("find", &rospack::find, this).doc(
			"Returns the name of the owner of this object.");
}

string rospack::find(string pack_name)
{

	return ros::package::getPath(pack_name);

}

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(rospack, "rospack")
