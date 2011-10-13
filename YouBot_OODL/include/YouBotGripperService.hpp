#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>
#include <youbot/YouBotGripper.hpp>

#include "YouBotTypes.hpp"
#include "YouBotOODL.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;
	using namespace boost::units;
	using namespace boost::units::si;

	/**
	 * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
	 */
	struct _GripperLimits
	{
		quantity<si::length> min_position;
		quantity<si::length> max_position;

		_GripperLimits() : min_position(0*meter), max_position(0*meter)
		{}
	};
	typedef struct _GripperLimits GripperLimits;

    class YouBotGripperService : public Service {

		public:
    		YouBotGripperService(const string& name, TaskContext* parent, unsigned int min_slave_nr);
			virtual ~YouBotGripperService();

//			void displayGripperStatus();

		protected:
			// Gripper
			InputPort<motion_control_msgs::JointPositions> gripper_cmd_position;
			OutputPort<sensor_msgs::JointState> gripper_state;

		private:
			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			bool check_error();

			// Gripper
	        motion_control_msgs::JointPositions m_gripper_cmd_position;
//	        sensor_msgs::JointState m_gripper_state;
	        GripperBarSpacingSetPoint m_tmp_gripper_cmd_position;
//	        GripperBarSpacingSetPoint m_tmp_gripper_state;
	        GripperLimits m_gripper_limits;

			YouBotGripper* m_gripper;

			bool m_calibrated;
    };

}
