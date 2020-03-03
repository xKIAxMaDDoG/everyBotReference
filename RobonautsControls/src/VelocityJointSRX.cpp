/*******************************************************************************
 *
 * File: VelocityJointSRX.cpp
 * 
 * Written by:
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/
#include <math.h>

#include "RobonautsLibrary/RobotUtil.h"
#include "RobonautsLibrary/XmlRobotUtil.h"
#include "gsu/Advisory.h"
#include "gsi/Time.h"
#include "RobonautsLibrary/DataLogger.h"
#include "RobonautsLibrary/MacroStepFactory.h"
#include "RobonautsLibrary/OIDriverStation.h"

#include "RobonautsControls/VelocityJointSRX.h"

#include "frc/smartdashboard/SmartDashboard.h" //WPI


using namespace tinyxml2;

/*******************************************************************************    
 * 
 * Create an instance of a servo control, feedback pot, and PID
 * and connect them to the specified inputs
 * 
 * @param    xml            the XML used to configure this object, see the class
 *                         definition for the XML format
 *                         
 * @param    period        the time (in seconds) between calls to update()
 * @param    priority    the priority at which this thread/task should run
 * 
 ******************************************************************************/
VelocityJointSRX::VelocityJointSRX(std::string control_name, XMLElement* xml)
    : PeriodicControl(control_name)
{
    XMLElement *comp;
    m_motor = nullptr;
    m_pid = nullptr;

    m_is_ready = false;

    m_increment_step = 0.0;
    m_decrement_step = 0.0;
    m_delta_position = 0.01;

    m_raw_position = -999.99;
    m_actual_position = -999.99;

    m_target_power = 0.0;
    m_target_position = 0.0;
    m_command_power = 0.0;
    m_max_power_delta = 0.2;
    m_num_setpoints = 0;


    m_position_adjust_rate = 0.0;

    m_setpoint_index = 0;
    for (uint8_t i = 0; i < NUM_SETPOINTS; i++)
    {
        m_setpoint[i] = 0.0;
    }


    m_closed_loop = false;
    m_target_power = 0.0;
    m_target_position = 0.0;

    //
	// Register Macro Steps
	//
	//new MacroStepProxy<MSPosJointSRXSetPower>(control_name, "SetPower", this);


 	//pjs_log = new DataLogger("/robot/logs/pjs", "pjs", "csv", 10, true);


	//
	// Parse the Controls XML
	//
    const char *name;
    xml->QueryBoolAttribute("closed_loop", &m_closed_loop);

	//xml->QueryFloatAttribute("max_velocity", &m_max_velocity);
	//xml->QueryFloatAttribute("desired_acceleration", &m_desired_acceleration);
	//xml->QueryFloatAttribute("desired_deceleration", &m_desired_deceleration);

	
	comp = xml->FirstChildElement("setpoints");
	if (comp != nullptr)
	{
 /*		while (setpoint_comp!=nullptr)
    	XMLElement *setpoint_comp;
		setpoint_comp = comp->FirstChildElement("setpoint");
		{
			int setpoint_index = -1;
			setpoint_comp->QueryIntAttribute("index", &setpoint_index);
			if (setpoint_index >= 0 && setpoint_index < VelocityJointSRX::NUM_SETPOINTS)
			{
				name = setpoint_comp->Attribute("name");
				if (name != nullptr)
				{
					m_setpoint_name[setpoint_index] = std::string(name);
				}
				else
				{
					m_setpoint_name[setpoint_index] = std::string("setpoint_") + std::to_string(setpoint_index);
					Advisory::pwarning("%s found unnamed setpoint, using default", getName());
				}

				setpoint_comp->QueryFloatAttribute("pjs", &m_pjs_setpoint[setpoint_index]);

				Advisory::pinfo("%s  -- setpoint %2d: %20s   pjs=%7.2f",getName(),
					setpoint_index, m_setpoint_name[setpoint_index].c_str(),
					m_pjs_setpoint[setpoint_index]);

					if (num_setpoints < setpoint_index + 1)
					{
						num_setpoints = setpoint_index + 1;
					}
			}
			else
			{
				Advisory::pinfo("%s setpoint with index out of range -- %d", getName(), setpoint_index);
			}
    		setpoint_comp = setpoint_comp->NextSiblingElement("setpoint");
		}*/
	}

	comp = xml-> FirstChildElement("motor");
	if (comp != NULL)
	{
		Advisory::pinfo("%s  creating speed controller", getName());
		m_motor = XmlRobotUtil::createRSpeedController(comp);
		if (m_closed_loop)
		{
		Advisory::pinfo("closed loop mode", getName().c_str());
			m_motor->SetControlMode(RSpeedController::POSITION);
		}
		else
		{
		Advisory::pinfo("open loop mode", getName().c_str());
			m_motor->SetControlMode(RSpeedController::DUTY_CYCLE);
		}
		m_motor->SetPosition(30.0); // set initial position to known value

	}
	else
	{
		Advisory::pwarning("%s  WARNING: required element \"motor\" not found", getName());
	}

	comp = xml-> FirstChildElement("oi");
	while (comp != NULL)
	{
		name = comp->Attribute("name");
		if (name != NULL)
		{
			if (strcmp(name, "analog") == 0)
			{
				Advisory::pinfo("%s  connecting analog channel", getName());
				OIController::subscribeAnalog(comp, this, CMD_MOVE_ANALOG);
//				comp->QueryFloatAttribute("delta_scale", &delta_position);
			}
			else if (strcmp(name, "setpoint_idx") == 0)
			{
				Advisory::pinfo("%s  connecting %s channel", getName(), name);
				OIController::subscribeInt(comp, this, CMD_SETPOINT_IDX);
			}
			else if ((strncmp(name, "setpoint", 8) == 0)
				&& (name[8] >= '0') && (name[8] <= '9'))
			{
				Advisory::pinfo("%s  connecting %s channel", getName(), name);
				OIController::subscribeDigital(comp, this, CMD_SETPOINT_0 + (name[8] - '0'));
			}
			else if ((strncmp(name, "momentary", 9) == 0)
					&& (name[9] >= '0') && (name[9] <= '3'))
			{
				int idx = name[9] - '0';

				Advisory::pinfo("%s  connecting %s channel [idx=%d]", getName(), name, idx);
//				comp->QueryFloatAttribute("ol_power", &(momentary_power[idx]));

//				Advisory::pinfo("%s  olp = %f, cls = %f", getName(), momentary_power[idx], momentary_step[idx] );

//				OIController::subscribeDigital(comp, this, CMD_MOMENTARY_0 + idx);
			}
			else if (strcmp(name, "closed_loop_state") == 0)
			{
				Advisory::pinfo("%s  connecting %s channel", getName(), name);
				OIController::subscribeDigital(comp, this, CMD_CLOSED_LOOP_STATE);
			}

		else
			{
				Advisory::pinfo("%s unknown oi %s", getName(), name);
			}
		}
		else
		{
			Advisory::pinfo("%s unnamed oi", getName());
		}
		comp = comp->NextSiblingElement("oi");
	}
}
/*******************************************************************************    
 * 
 * Release any resources allocated by this object
 * 
 ******************************************************************************/
VelocityJointSRX::~VelocityJointSRX(void)
{
    if (m_motor != nullptr)
    {
        delete m_motor;
        m_motor = nullptr;
    }

    if (m_pid != nullptr)
    {
        delete m_pid;
        m_pid = nullptr;
    }
}

/*******************************************************************************    
 *
 *
 *
 ******************************************************************************/
void VelocityJointSRX::controlInit(void)
{
    bool is_ready = true;

    if (m_pid == nullptr)
    {
        Advisory::pwarning("  WARNING: VelocityJointSRX element \"pid\" not found, cannot use closed loop");
 //       is_ready = false;
    }

    m_is_ready = is_ready;
}

/*******************************************************************************
 *
 *
 *
 ******************************************************************************/
void VelocityJointSRX::updateConfig(void)
{

}

/*******************************************************************************
 *
 * This is the callback for OIController::subscribeAnalog, if the XML config 
 * specifies an analog input, the constructor of this object will connect that 
 * input to this method.
 * 
 * @param id    the control id passed to the subscribe
 * @param val    the new value of the analog channel that was subscribed to
 * 
 ******************************************************************************/
void VelocityJointSRX::setAnalog(int id, float val)
{
    Advisory::pinfo("%s(%d, %f)", __FUNCTION__, id, val);
    if (id == CMD_MOVE_ANALOG)
    {
        if (m_closed_loop == true)
        {
//            target_position += (val * delta_position);
//see position_ajust_rate in doPeriodic
            m_position_adjust_rate = (val);
        }
        else
        {
            m_target_power = RobotUtil::limit(-1.0, 1.0, val);
        }
    }
}

/*******************************************************************************    
 *
 * This is the callback for OIController::setDigital, if the XML config 
 * specifies an increment, decrement, or stop input, the constructor 
 * of this object will connect that/those input(s) to this method.
 * 
 * @param id    the control id passed to the subscribe
 * @param val    the new value of the digital channel that was subscribed to
 * 
 ******************************************************************************/
void VelocityJointSRX::setDigital(int id, bool val)
{
    Advisory::pinfo("VelocityJointSRX::setDigital(%d, %s)", id, val?"true":"false");
    switch (id)
    {
        case CMD_CLOSED_LOOP_STATE:
        {
            m_closed_loop = val;
        } break;

        case CMD_SETPOINT_0:     applySetpoint(val, 0); break;
        case CMD_SETPOINT_1:     applySetpoint(val, 1); break;
        case CMD_SETPOINT_2:     applySetpoint(val, 2); break;
        case CMD_SETPOINT_3:     applySetpoint(val, 3); break;

        case CMD_INCREMENT_POS:
        {
            if (m_setpoint_index < NUM_SETPOINTS - 1)
            {
                applySetpoint(val, m_setpoint_index + 1);
            }
            else
            {
                applySetpoint(val, NUM_SETPOINTS - 1);
            }
        } break;

        case CMD_DECREMENT_POS:
        {
            if (m_setpoint_index > 0)
            {
                applySetpoint(val, m_setpoint_index - 1);
            }
            else
            {
                applySetpoint(val, 0);
            }
        } break;

        case CMD_UPDATE_SETPOINT:
        {
        	if (val)
        	{
        		m_setpoint[m_setpoint_index]=m_actual_position;
        		Advisory::pinfo("VelocityJointSRX setpoint %d changed to %f", m_setpoint_index, m_actual_position);
        	}
        } break;

        default:
            break;
    }
}

/*******************************************************************************    
 *
 * This is the callback for OIController::subscribeInt, if the XML config
 * specifies an int, the constructor of this object will connect that
 * input to this method.
 *
 * @param id    the control id passed to the subscribe
 * @param val    the new value of the int channel that was subscribed to
 *
 ******************************************************************************/
void VelocityJointSRX::setInt(int id, int val)
{
   
}
/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::applySetpoint(bool on, int idx)
{
    if (on && (idx >= 0) && (idx < m_num_setpoints))
    {
        m_setpoint_index = idx;
        m_target_position = m_setpoint[idx];
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::setPosition(float val)
{
    m_target_position = val;
}

/*******************************************************************************
 *
 ******************************************************************************/
float VelocityJointSRX::getPosition(void)
{
    return m_actual_position;
}
/*******************************************************************************
 *
 ******************************************************************************/
float VelocityJointSRX::getTargetPosition(void)
{
    return m_target_position;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::disabledInit()
{
    if(nullptr != m_vjs_log)
    {
        m_vjs_log->close();
    }

}

/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::autonomousInit()
{
    initLogFile("auton");
}

/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::teleopInit()
{
   // VelocityJointSRX::initLogFile();
}
/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::testInit()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void VelocityJointSRX::publish()
{
    SmartDashboard::PutBoolean(std::string("  ") + getName() + "  ", (getCyclesSincePublish() > 0));
    SmartDashboard::PutNumber(getName() + " raw_position: ", m_raw_position);

    SmartDashboard::PutNumber(getName() + " actual position: ", m_actual_position);
    SmartDashboard::PutNumber(getName() + " target position: ", m_target_position);

#ifdef DEBUG_PUBLISH
    if (m_pid != nullptr)
    {
        SmartDashboard::PutNumber(getName() +" PID Ep: ", m_pid->getErrorP());
        SmartDashboard::PutNumber(getName() +" PID Ei: ", m_pid->getErrorI());
        SmartDashboard::PutNumber(getName() +" PID Ed: ", m_pid->getErrorD());
    }

    SmartDashboard::PutNumber(getName() +" cycles: ", getCyclesSincePublish());
    SmartDashboard::PutBoolean(getName() + " is ready: ", m_is_ready);

    SmartDashboard::PutBoolean(getName() +" closed loop: ", m_closed_loop);


    SmartDashboard::PutNumber(getName() +" target power: ", m_target_power);
    SmartDashboard::PutNumber(getName() +" command power: ", m_command_power);
    SmartDashboard::PutNumber(getName() +" set point idx: ", m_setpoint_index);
#endif

}

/*******************************************************************************
 *
 ******************************************************************************/
bool VelocityJointSRX::isClosedLoop(void)
{
    return m_closed_loop;
}

/**********************************************************************
 *
 * This method is used to initialize the log files, called from
 * teleop init and auton init methods
 *
 **********************************************************************/
void VelocityJointSRX::initLogFile(std::string phase)
{
    if(0==strcmp(phase.c_str(), "teleop"))
    {
        m_vjs_log = m_vjs_teleop_log;
    }
    else
    {
        m_vjs_log = m_vjs_auton_log;
    }
    if(nullptr != m_vjs_log)
    {
        m_vjs_log->openSegment();
        m_vjs_log->log("time, battery,");
        m_vjs_log->log("target_position, actual_position, raw_position,");

        m_vjs_log->log("target_power,command_power,");
        m_vjs_log->log("setpoint_index,");

        for(int i=0;i<NUM_SETPOINTS;i++)
        {
            m_vjs_log->log("setpoint[%d],",i);
        }
        m_vjs_log->log("\n");
    }
}

/**********************************************************************
 *
 * Log variables from do periodic
 *
**********************************************************************/
void VelocityJointSRX::logVariables()
{
    // write to log
    if(nullptr != m_vjs_log)
    {
        m_vjs_log->log("%8.6f, %8.6f,", getPhaseElapsedTime(), DriverStation::GetInstance().GetBatteryVoltage());
        m_vjs_log->log("%8.6f, %8.6f, %8.6f,", m_target_position, m_actual_position, m_raw_position);

        m_vjs_log->log("%8.6f,%8.6f,", m_target_power,m_command_power);
        m_vjs_log->log("%d,",m_setpoint_index);

        for(int i=0;i<NUM_SETPOINTS;i++)
        {
            m_vjs_log->log("%8.6f, ", m_setpoint[i]);
        }
        m_vjs_log->log("\n");
        m_vjs_log->flush();
    }
}


/*******************************************************************************
 *
 * This method will be called once a period to do anything that is needed,
 * in this case it just sets the servo power to the current value.
 * 
 ******************************************************************************/
void VelocityJointSRX::doPeriodic()
{
    if (m_is_ready == false)
    {
        return;
    }

    if (m_closed_loop)
    {
        Advisory::pinfo("closed loop");
    }
    else
    {
        m_target_position = m_actual_position;
         Advisory::pinfo("actual position = %f", m_actual_position);
   }

    if (getPhase() == DISABLED)
    {
        m_target_position = m_actual_position;
        m_target_power = 0.0;
        m_command_power = 0.0;
    }

    //
    // Ramp the power changes
    //
    if (m_command_power + m_max_power_delta < m_target_power)
    {
        m_command_power += m_max_power_delta;
    }
    else if (m_command_power - m_max_power_delta > m_target_power)
    {
        m_command_power -= m_max_power_delta;
    }
    else
    {
        m_command_power = m_target_power;
    }

    //
    // Set output power
    //
    if (m_motor != nullptr)
    {
        m_motor->Set(m_command_power);
    }
    logVariables();

}

/*******************************************************************************
 *
 ******************************************************************************/
MSVelJointSRXSetPosition::MSVelJointSRXSetPosition(std::string type, tinyxml2::XMLElement *xml, void *control)
    : MacroStepSequence(type, xml, control)
{
    m_parent_control = (VelocityJointSRX *)control;
    m_position=0.0;
    xml->QueryFloatAttribute("position", &m_position);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MSVelJointSRXSetPosition::init(void)
{
    m_parent_control->setPosition(m_position);
}

/*******************************************************************************
 *
 ******************************************************************************/
MacroStep * MSVelJointSRXSetPosition::update(void)
{
    return next_step;
}

/*******************************************************************************
 *
 ******************************************************************************/
MSVelJointSRXIsReady::MSVelJointSRXIsReady(std::string type, tinyxml2::XMLElement *xml, void *control)
    : MacroStepCondition(type, xml, control)
{
    m_parent_control = (VelocityJointSRX *)control;
    m_position_tolerance=100.0;
    xml->QueryFloatAttribute("position_tolerance", &m_position_tolerance);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MSVelJointSRXIsReady::init(void)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
MacroStep * MSVelJointSRXIsReady::update(void)
{
    if (fabs(m_parent_control->getTargetPosition() - m_parent_control->getPosition()) < m_position_tolerance)
    {
        return true_step;
    }
    else
    {
        return false_step;
    }
}

