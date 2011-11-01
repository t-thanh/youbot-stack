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
void Top::e_guardedMove()
{
	setState<guardedMove>();
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
void guardedMove::init()
{
	error.resize(6,0);
}
void guardedMove::run(YouBot::YouBot_executive* executive)
{
	//RTT::log(Info)<<"guardedMove"<<RTT::endlog();
	vector<double> setPoint_j;
	vector<double> setPoint_c;
	vector<double> states_j;
	vector<double> states_c;
	vector<double> stiffness_j;
	vector<double> stiffness_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
	vector<double> guardForce;
	vector<double> stateForce;
	double  alpha=0.1;
	bool done;
	executive->getSetPoints(setPoint_j,setPoint_c);
	executive->getStates(states_j,states_c);
	executive->getStiffness(stiffness_j, stiffness_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->getGuardForce(guardForce);
	executive->getStateForce(stateForce);
//	RTT::log(Info)<<"all data read"<<RTT::endlog();
	done=true;
//	RTT::log(Info)<<"stateForce "<<stateForce.size();
//	RTT::log(Info)<<"guardForce "<<guardForce.size();
//	RTT::log(Info)<<"setPoint_c "<<setPoint_c.size();

	for(std::size_t i=0; i<3;i++)
	{
		if (guardForce.at(i)!=0)
		{
			error.at(i)=guardForce.at(i);///stiffness_c.at(1);
			done=done and (abs(guardForce.at(i)+stateForce.at(i))<alpha);
			setPoint_c.at(i)=states_j.at(i)+error.at(i);
		}
	//	RTT::log(Info)<<"\t "<<i<<"="<<error.at(i);
	}
//	RTT::log(Info)<<RTT::endlog();
//	RTT::log(Info)<<"error computed"<<RTT::endlog();
	if (done)
	{
		executive->doneEvent();
		executive->positionArm(states_j);
	}
//	RTT::log(Info)<<"Position set "<<RTT::endlog();
	executive->writeSetpoints(states_j,zeroStiffness_j,setPoint_c,stiffness_c);
}
const double retractGripper::STIFFNESS_C[]={50,150};
const double retractGripper::GRIPPER_SIZE[]={0,0,2};
void retractGripper::init(){
	setPoint.clear();
	vecStiffness.assign(STIFFNESS_C,STIFFNESS_C+2);
	vecGripperSize.assign(GRIPPER_SIZE,GRIPPER_SIZE+3);
	firstRun=true;
}
void retractGripper::run(YouBot::YouBot_executive* executive)
{
	vector<double> setPoint_j;
	vector<double> setPoint_c;
	vector<double> states_j;
	vector<double> states_c;
	vector<double> stiffness_j;
	vector<double> stiffness_c;
	vector<double> zeroStiffness_j;
	vector<double> zeroStiffness_c;
	vector<double> vecH;

	executive->getSetPoints(setPoint_j,setPoint_c);
	executive->getStates(states_j,states_c);
	executive->getStiffness(stiffness_j, stiffness_c);
	executive->getZeroStiffness(zeroStiffness_j,zeroStiffness_c);
	executive->getGripperH(vecH);
	if (firstRun){
		vector<double> error;
		Multiply(vecH,vecGripperSize,error);
		Sum(setPoint_c,error, setPoint);
		firstRun=false;
	}
	executive->writeSetpoints(states_j,zeroStiffness_j,setPoint,vecStiffness);

}
std::string Top::toString()
{
	return "Top";
}
std::string fullControl::toString()
{
	return "fullControl";
}
std::string catesianControl::toString()
{
	return "catesianControl";
}
std::string gravityMode::toString()
{
	return "gravityMode";
}
std::string jointControl::toString()
{
	return "jointControl";
}
std::string guardedMove::toString()
{
	return "guardedMove";
}





std::string retractGripper::toString(){return "retractGripper";}

} //flowControl

void Multiply(const vector<double>& H,const vector<double>& r, vector<double>& output)
{
	using namespace std;

	if(H.size()!=16 && r.size()!=3)
	{
		 __throw_out_of_range(__N("Multiply::vector::_M_range_check"));
	return;
	}
	output.clear();
	output.resize(3,0);
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			output[i]+=H[i*4+j]*r[j];
		}
	}
}
void Sum(const vector<double>& rhs,const vector<double>& lhs, vector<double>& output)
{
	using namespace std;

	if(lhs.size()!=rhs.size())
	{
		 __throw_out_of_range(__N("Sum::vector::_M_range_check"));
	return;
	}
	output.clear();
	output.resize(lhs.size(),0);
	for(int i=0;i<lhs.size();i++)
	{

			output[i]=lhs[i]*rhs[i];

	}
}
