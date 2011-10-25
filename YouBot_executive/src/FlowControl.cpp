#include "FlowControl.hpp"
#include "YouBot_executive.h"
namespace FlowControl
{
using namespace YouBot;
void Top::init()
{
   setState<fullControl>();
}
void Top::e_fullControl()
{
	setState<fullControl>();
}
void Top::e_gravityMode()
{
	setState<gravityMode>();
}
void Top::e_jointControl()
{
	setState<jointControl>();
}
void Top::e_catesianControl()
{
	setState<catesianControl>();
}
void fullControl::run(YouBot::YouBot_executive* executive)
{
	vector<double> setPoint_j;
	vector<double> setPoint_c;
	vector<double> states_j;
	vector<double> states_c;
	vector<double> stiffness_j;
	vector<double> stiffness_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
    executive->getSetPoints(setPoint_j,setPoint_c);
	executive->getStates(states_j,states_c);
	executive->getStiffness(stiffness_j, stiffness_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->writeSetpoints(setPoint_j,stiffness_j,setPoint_c,stiffness_c);
}
void gravityMode::run(YouBot::YouBot_executive* executive)
{
	vector<double> states_j;
	vector<double> states_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
	executive->getStates(states_j,states_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->writeSetpoints(states_j,zeroStiffness_j,states_c,zeroStiffness_c);

}

void jointControl::run(YouBot::YouBot_executive* executive)
{
	vector<double> setPoint_j;
	vector<double> setPoint_c;
	vector<double> states_j;
	vector<double> states_c;
	vector<double> stiffness_j;
	vector<double> stiffness_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
	executive->getSetPoints(setPoint_j,setPoint_c);
	executive->getStates(states_j,states_c);
	executive->getStiffness(stiffness_j, stiffness_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->writeSetpoints(setPoint_j,stiffness_j,states_c,zeroStiffness_c);

}
void catesianControl::run(YouBot::YouBot_executive* executive)
{
	vector<double> setPoint_j;
	vector<double> setPoint_c;
	vector<double> states_j;
	vector<double> states_c;
	vector<double> stiffness_j;
	vector<double> stiffness_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
	executive->getSetPoints(setPoint_j,setPoint_c);
	executive->getStates(states_j,states_c);
	executive->getStiffness(stiffness_j, stiffness_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->writeSetpoints(states_j,zeroStiffness_j,setPoint_c,stiffness_c);
}

} //flowControl
