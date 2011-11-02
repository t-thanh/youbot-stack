#include "YouBot_configs.hpp"
namespace YouBot
{
	YouBot_config* YouBot_config::m_instance=0;
	 const double YouBot_config::UNFOLD_JOINT_POSE[]={0,0,0,0,0,0,0,0};
	 const double YouBot_config::SNAKE_JOINT_POSE[]={0,0,0,0,-0.6,0.7,0.7,0};

	 const double YouBot_config::LOW_JOINT_STIFFNESS[]={0,0,0,0,0,0,0,0};
	 const double YouBot_config::LOW_CARTESIAN_STIFFNESS[]={0,0};

	 const double YouBot_config::MEDIUM_JOINT_STIFFNESS[]={15,15,15,30,30,30,30,30};
	 const double YouBot_config::MEDIUM_CARTESIAN_STIFFNESS[]={5,70};

	 const double YouBot_config::HIGH_JOINT_STIFFNESS[]={50,50,50,100,100,100,100,100};
	 const double YouBot_config::HIGH_CARTESIAN_STIFFNESS[]={50,500};

	  YouBot_config *YouBot_config::instance()
	    {
	        if (!m_instance)
	          m_instance = new YouBot_config;
	        return m_instance;
	    }

	YouBot_config::YouBot_config(){

		 m_unfoldJointPose.assign(UNFOLD_JOINT_POSE,UNFOLD_JOINT_POSE+SIZE_JOINTS_ARRAY);
		 m_snakeJointPose.assign(SNAKE_JOINT_POSE,SNAKE_JOINT_POSE+SIZE_JOINTS_ARRAY);

		 m_lowJointStiffnes.assign(LOW_JOINT_STIFFNESS,LOW_JOINT_STIFFNESS+SIZE_JOINTS_ARRAY);
		 m_lowCartesianStiffnes.assign(LOW_CARTESIAN_STIFFNESS,LOW_CARTESIAN_STIFFNESS+SIZE_CARTESIAN_STIFFNESS);

		 m_mediumJointStiffnes.assign(MEDIUM_JOINT_STIFFNESS,MEDIUM_JOINT_STIFFNESS+SIZE_JOINTS_ARRAY);
		 m_mediumCartesianStiffnes.assign(MEDIUM_CARTESIAN_STIFFNESS,MEDIUM_CARTESIAN_STIFFNESS+SIZE_CARTESIAN_STIFFNESS);

		 m_highJointStiffnes.assign(HIGH_JOINT_STIFFNESS,HIGH_JOINT_STIFFNESS+SIZE_JOINTS_ARRAY);
		 m_highCartesianStiffnes.assign(HIGH_CARTESIAN_STIFFNESS,HIGH_CARTESIAN_STIFFNESS+SIZE_CARTESIAN_STIFFNESS);;
	}
	YouBot_config::~YouBot_config(){}

	std::vector<double> YouBot_config::getJointPose(YouBot_config::JointPoses pose)
    {
    	switch(pose){
    	case UNFOLD :
    		return m_unfoldJointPose;
    		break;
    	case SNAKE:
    		 return m_snakeJointPose;
    		break;
    	default:
    		std::__throw_out_of_range(__N("getJointPose::enum::_M_range_check"));

    		break;
    	}
    	return std::vector<double>();
    }


	std::vector<double> YouBot_config::getJointStiffness(YouBot_config::StiffnessValue value)
    {
    	switch(value){
    	case HIGH :
    		return m_highJointStiffnes;
    		break;
    	case MEDIUM:
    		 return m_mediumJointStiffnes;
    		break;
    	case LOW:
    		 return m_lowJointStiffnes;
    		 break;
    	default:
    		std::__throw_out_of_range(__N("getJointStiffness::enum::_M_range_check"));

    		break;
    	}
    	return std::vector<double>();
    }
	std::vector<double> YouBot_config::getCartesianStiffness(YouBot_config::StiffnessValue value)
    {
    	switch(value){
    	case HIGH :
    		return m_highCartesianStiffnes;
    		break;
    	case MEDIUM:
    		 return m_mediumCartesianStiffnes;
    		break;
    	case LOW:
    		 return m_lowCartesianStiffnes;
    		 break;
    	default:
    		std::__throw_out_of_range(__N("getCartesianStiffness::enum::_M_range_check"));

    		break;
    	}
    	return std::vector<double>();
    }




} //namespace YouBot
