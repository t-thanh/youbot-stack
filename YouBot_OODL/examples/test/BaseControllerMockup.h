#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include "YouBotTypes.h"
#include <boost/units/systems/si.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace boost::units;
	using namespace boost::units::si;

	class BaseControllerMockup: public TaskContext
	{
		public:
		BaseControllerMockup(const string& name);
		virtual ~BaseControllerMockup();

		void getBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation);
		void setBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation);

		void getBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity);
		void setBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity);

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
			void setJointAngles(vector< double >& angles, double epsilon);

			InputPort< sensor_msgs::JointState > joint_states;

			InputPort< vector<joint_status> > joint_statuses;

			OutputPort< motion_control_msgs::JointPositions > joint_cmd_angles;
			OutputPort< vector<ctrl_modes> > joint_ctrl_modes;

			sensor_msgs::JointState m_joint_states;

			vector<joint_status> m_joint_statuses;
			vector<ctrl_modes> m_modes;

			motion_control_msgs::JointPositions m_joint_cmd_angles;
//			vector<quantity<si::angular_velocity> > m_joint_cmd_velocities;
//			vector<quantity<si::torque> > m_joint_cmd_torques;

			OperationCaller<void(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)> op_getBasePosition;
			OperationCaller<void(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)> op_setBasePosition;

			OperationCaller<void(quantity<velocity>& longitudinalPosition, quantity<velocity>& transversalPosition, quantity<angular_velocity>& orientation)> op_setBaseVelocity;

	};
}
