#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotManipulator.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace youbot;

	class YouBot_configurator: public TaskContext
	{
	public:
		YouBot_configurator(const string& name);
		virtual ~YouBot_configurator();

		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();

	private:
		YouBotManipulator* m_manipulator;
		YouBotBase* m_base;
	};
}

