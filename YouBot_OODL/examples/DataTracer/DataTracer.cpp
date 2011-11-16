#include "DataTracer.hpp"

#include <ocl/Component.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;
	using namespace boost::units;
	using namespace boost::units::si;

	DataTracer::DataTracer(const string& name) :
			TaskContext(name, PreOperational)
	{
		m_setpoint.data.resize(1);

		this->addPort("setpoint",setpoint).doc("The given setpoint");

		this->addOperation("setJoint",&DataTracer::setJoint,this);
		this->addOperation("getJoint",&DataTracer::getJoint,this);
		this->addOperation("setpointType",&DataTracer::setpointType,this);

		GlobalsRepository::shared_ptr globals = GlobalsRepository::Instance();
		globals->setValue( new Constant<my_part>("ARM",DataTracer::ARM) );
		globals->setValue( new Constant<my_part>("BASE",DataTracer::BASE) );
	}

	DataTracer::~DataTracer() {}

	unsigned int DataTracer::getJoint()
	{
		return m_joint_nr;
	}

	bool DataTracer::setJoint(my_part part, unsigned int joint_nr)
	{
		if(this->getTaskState() != PreOperational)
		{
			return false;
		}

		m_part = part;
		m_joint_nr = joint_nr;
		return true;
	}

	bool DataTracer::setpointType(unsigned int setp)
	{
		if(this->getTaskState() != PreOperational)
		{
			return false;
		}

		m_setpoint_type = setp;
		return true;
	}

	bool DataTracer::configureHook()
	{
		try
		{
			if(m_part == ARM)
			{
				YouBotManipulator* manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
				if(manipulator == NULL)
				{
					log(Error) << "Could not create the YouBotManipulator." << endlog();
					return false;
				}

				m_trace = new DataTrace((manipulator->getArmJoint(m_joint_nr)));
			}
			else if(m_part == BASE)
			{
				YouBotBase* base = new YouBotBase("/youbot-base", OODL_YOUBOT_CONFIG_DIR);
				if(base == NULL)
				{
					log(Error) << "Could not create the YouBotBase." << endlog();
					return false;
				}
				m_trace = new DataTrace((base->getBaseJoint(m_joint_nr)));
			}
			else
			{
				log(Error) << "YouBot part not recognized." << endlog();
			}
		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			this->error();
			return false;
		}

		return TaskContext::configureHook();
	}

	bool DataTracer::startHook()
	{
		if(!setpoint.connected())
		{
			return false;
		}

		m_trace->startTrace();
		return TaskContext::startHook();
	}

	void DataTracer::updateHook()
	{
        TaskContext::updateHook();

        setpoint.read(m_setpoint);

        switch(m_setpoint_type)
        {
        	case(ANGULAR):
			{
        		JointAngleSetpoint sp;
        		sp.angle = m_setpoint.data[0]* si::radian;
        		m_trace->updateTrace(sp);
        		break;
			}
        	case(VELOCITY):
			{
        		JointVelocitySetpoint sp;
        		sp.angularVelocity = m_setpoint.data[0]* si::radian_per_second;
        		m_trace->updateTrace(sp);
        		break;
			}
        	case(CURRENT):
			{
        		JointCurrentSetpoint sp;
        		sp.current = m_setpoint.data[0]* si::ampere;
        		m_trace->updateTrace(sp);
        		break;
			}
        	case(TORQUE):
			{
        		JointTorqueSetpoint sp;
        		sp.torque = m_setpoint.data[0] * si::newton_meter;
        		m_trace->updateTrace(sp);
        		break;
			}
        	case(PWM):
			{
        		JointPWMSetpoint sp;
        		sp.pwm = m_setpoint.data[0];
        		m_trace->updateTrace(sp);
        		break;
			}
        	default:
        	{
        		log(Error) << "DataTracer - setpoint type not recognized." << endlog();
        		this->error();
        		break;
        	}
        }
	}

	void DataTracer::stopHook()
	{
		m_trace->stopTrace();
	}

}

ORO_CREATE_COMPONENT( YouBot::DataTracer )
