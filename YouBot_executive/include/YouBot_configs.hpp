#pragma once
#include <vector>
namespace YouBot
{
/**
 * The class is Singleton  to allow future extension of configuration parameters to be read from file.
 * This is needed so reading happens  only once
 */
class YouBot_config
{
public:

	~YouBot_config();
	enum StiffnessValue {LOW,MEDIUM,HIGH} ;
	enum JointPoses {UNFOLD,SNAKE} ;
	static const int SIZE_JOINTS_ARRAY = 8; //5 arm + 3  virtual base
	static const int SIZE_CARTESIAN_SPACE = 6; //
	static const int SIZE_CARTESIAN_STIFFNESS = 2;
	static const double GRIPPER_OPENING = 0.022;
	std::vector<double> getJointPose(JointPoses pose);
	std::vector<double> getJointStiffness(StiffnessValue value) ;
	std::vector<double> getCartesianStiffness(StiffnessValue value) ;
	static YouBot_config *instance();

private:
	static YouBot_config* m_instance;
	YouBot_config();
	static const double UNFOLD_JOINT_POSE[SIZE_JOINTS_ARRAY];
	static const double SNAKE_JOINT_POSE[SIZE_JOINTS_ARRAY];

	static const double LOW_JOINT_STIFFNESS[SIZE_JOINTS_ARRAY];
	static const double LOW_CARTESIAN_STIFFNESS[SIZE_CARTESIAN_STIFFNESS];

	static const double MEDIUM_JOINT_STIFFNESS[SIZE_JOINTS_ARRAY];
	static const double MEDIUM_CARTESIAN_STIFFNESS[SIZE_CARTESIAN_STIFFNESS];

	static const double HIGH_JOINT_STIFFNESS[SIZE_JOINTS_ARRAY];
	static const double HIGH_CARTESIAN_STIFFNESS[SIZE_CARTESIAN_STIFFNESS];

	std::vector<double> m_unfoldJointPose;
	std::vector<double> m_snakeJointPose;

	std::vector<double> m_lowJointStiffnes;
	std::vector<double> m_lowCartesianStiffnes;

	std::vector<double> m_mediumJointStiffnes;
	std::vector<double> m_mediumCartesianStiffnes;

	std::vector<double> m_highJointStiffnes;
	std::vector<double> m_highCartesianStiffnes;

};
} //namespace YouBot
