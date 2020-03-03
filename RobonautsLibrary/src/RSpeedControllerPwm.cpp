/*******************************************************************************
 *
 * File: RAbsPosSensor.cpp
 * 
 * This file contains the definition of a class for interacting with an
 * Absolute Position Sensor.
 *
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#include "gsu/Advisory.h"
#include "RobonautsLibrary/RSpeedControllerPwm.h"

/******************************************************************************
 *
 * Create an interface of RSpeedControllerCan, get an APS plugged into the specified channel.
 *
 * @param   channel the channel or port that the pot is in
 *
 ******************************************************************************/
RSpeedControllerPwm::RSpeedControllerPwm(frc::PWMSpeedController *sc)
    : RSpeedController()
      , m_controller(sc)
{
    m_fuse = 0;

    m_current_limit = 999.999;
    m_current_limit_enabled = false;

    m_mode = ControlModeType::UNKNOWN_TYPE;
}

RSpeedControllerPwm::~RSpeedControllerPwm()
{
    if (m_controller != nullptr)
    {
        delete m_controller;
        m_controller = nullptr;
    }
}

void RSpeedControllerPwm::ProcessXML(tinyxml2::XMLElement *xml) {
    bool invert_motor = false;
    int fuse = -1;

    xml->QueryBoolAttribute("invert", &invert_motor);
    InvertMotor(invert_motor);

    xml->QueryIntAttribute("fuse", &fuse);
    SetFuse(fuse);

    Advisory::pinfo("    creating Speed Controller on port %d, invert_m=%s", m_controller->GetChannel(), invert_motor?"true":"false");
}

bool RSpeedControllerPwm::SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a, int port_b)
{
    bool ret_val = true;

    switch(sensor_type)
    {
        case INTERNAL_ENCODER:
            ret_val = false;
            break;
        case INTERNAL_POTENTIOMETER:
            ret_val = false;
            break;
        case EXTERNAL_ENCODER:
            // create encoder using A/B, set duty cycle mode
            break;
        case EXTERNAL_POTENTIOMETER:
            // creat rpot
            break;
        default:
            ret_val = false;
            break;
    }
    return(ret_val);
}

bool RSpeedControllerPwm::InvertMotor(bool invert)
{
    m_controller->SetInverted(invert);
    return(true);
}

bool RSpeedControllerPwm::SetControlMode(ControlModeType type, int deviceID) 
{ 
    return(false); 
}

void RSpeedControllerPwm::InvertSensor(bool invert, bool is_drv) { m_sensor_invert = invert ? -1.0 : 1.0 ; }

void RSpeedControllerPwm::Set(double val) { m_controller->Set(val); }
double RSpeedControllerPwm::Get() { return  m_controller->Get(); }

void RSpeedControllerPwm::SetPosition(double pos) {} 
double RSpeedControllerPwm::GetPosition() { return(0.0); }
int32_t RSpeedControllerPwm::GetRawPosition() { return(0.0); }

double RSpeedControllerPwm::GetSpeed() { return(0.0); }
int32_t RSpeedControllerPwm::GetRawSpeed() { return 0; }

int RSpeedControllerPwm::GetDeviceID() { return(-1); }

void RSpeedControllerPwm::SetCurrentLimit(uint32_t amps, uint32_t peak, uint32_t duration ) { m_current_limit = amps; }
void RSpeedControllerPwm::SetCurrentLimitEnabled(bool enabled) { m_current_limit_enabled = enabled; }
void RSpeedControllerPwm::SetForwardLimitSwitch (bool normally_open, bool enabled, bool zero_position) {}
void RSpeedControllerPwm::SetReverseLimitSwitch  (bool normally_open, bool enabled, bool zero_position) {}

double RSpeedControllerPwm::GetBusVoltage() { return m_power_panel.GetVoltage(); }
double RSpeedControllerPwm::GetOutputVoltage() { return 0.0; }
double RSpeedControllerPwm::GetMotorOutputPercent() { return m_controller->Get(); }
double RSpeedControllerPwm::GetOutputCurrent() { return m_power_panel.GetCurrent(m_fuse); }
double RSpeedControllerPwm::GetActiveTrajectoryPosition() { return 0; }
double RSpeedControllerPwm::GetActiveTrajectoryVelocity() { return 0; }

void RSpeedControllerPwm::SetP(double p) {}
void RSpeedControllerPwm::SetI(double i) {}
void RSpeedControllerPwm::SetD(double d) {}
void RSpeedControllerPwm::SetF(double f) {}
void RSpeedControllerPwm::SetIZone(double i_zone) {}
void RSpeedControllerPwm::SetCruiseVelocity(double vel) {}
void RSpeedControllerPwm::SetAcceleration(double accel) {}

double RSpeedControllerPwm::GetP() { return 0.0; }
double RSpeedControllerPwm::GetI() { return 0.0; }
double RSpeedControllerPwm::GetD() { return 0.0; }
double RSpeedControllerPwm::GetF() { return 0.0; }

void RSpeedControllerPwm::SetFuse(int fuse) { m_fuse = fuse; }

