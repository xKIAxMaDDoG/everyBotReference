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
#include "frc/shuffleboard/Shuffleboard.h"
#include "RobonautsLibrary/RSpeedControllerSparkMaxCan.h"
#include "gsu/Advisory.h"
#include "RobonautsLibrary/RobotUtil.h"

/******************************************************************************
 *
 * Create an interface of RSpeedControllerCan, get an APS plugged into the specified channel.
 *
 * @param   channel the channel or port that the pot is in
 *
 ******************************************************************************/
RSpeedControllerSparkMaxCan::RSpeedControllerSparkMaxCan(int port_device)
    : RSpeedController(port_device)
    , m_controller(port_device, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_device_id(port_device)
    , m_pidController(m_controller.GetPIDController())
    , m_encoder(m_controller.GetEncoder())
{
   m_pidController.SetOutputRange(-1.0, 1.0);
}

RSpeedControllerSparkMaxCan::~RSpeedControllerSparkMaxCan() {}

void RSpeedControllerSparkMaxCan::ProcessXML(tinyxml2::XMLElement *xml) {

    tinyxml2::XMLElement *elem;
    elem = xml->FirstChildElement("encoder");
    if (elem != nullptr)
    {
        //SetFeedbackDevice(RSpeedControllerSparkMaxCan :: );
        double scale = 1.0;
        elem->QueryDoubleAttribute("scale", &scale);
        SensorOutputPerCount(scale);
    }

    double kf = 0.001;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double iz = 0.0;
    double cruise_velocity = 0.0;
    double acceleration = 0.0;

    elem = xml->FirstChildElement("pid");
    if (elem != nullptr)
    {
        elem->QueryDoubleAttribute("kf", &kf);
        elem->QueryDoubleAttribute("kp", &kp);
        elem->QueryDoubleAttribute("ki", &ki);
        elem->QueryDoubleAttribute("kd", &kd);
        elem->QueryDoubleAttribute("iz", &iz);
        elem->QueryDoubleAttribute("cruise_velocity", &cruise_velocity);
        elem->QueryDoubleAttribute("acceleration", &acceleration);

        SetF(kf);
        SetP(kp);
        SetI(ki);
        SetD(kd);

        if (iz != 0.0)
        {
            SetIZone(iz);
        }

        if (cruise_velocity != 0.0)
        {
            SetCruiseVelocity(cruise_velocity);
        }

        if (acceleration != 0.0)
        {
            SetAcceleration(acceleration);
        }
    }

    bool invert_motor = false;
    bool brake_mode = false;

    xml->QueryBoolAttribute("invert", &invert_motor);
    InvertMotor(invert_motor);

    xml->QueryBoolAttribute("brake_mode", &brake_mode);
    SetBrakeMode(brake_mode);
}

/******************************************************************************
 *
 *
 * If sensor_type is INTERNAL_POTENTIOMETER:
 *     port_a==0 means no sensor wrap, port_a!=0 means sensor wrap
 *
 ******************************************************************************/
bool RSpeedControllerSparkMaxCan::SetFeedbackDevice(FeedbackDeviceType sensor_type, int ticks_per_rev, int port_b)
{
    bool ret_val = false;

    switch(sensor_type)
    {
        case INTERNAL_BRUSHLESS_HALL:  // SparkMax "encoder" generated using hall effect sensors
            m_encoder = m_controller.GetEncoder();
            m_pidController.SetFeedbackDevice(m_encoder);
            ret_val = true;
            break;

        case INTERNAL_ENCODER:    // alternate encoder wired into limit switch ports
            m_encoder = m_controller.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, ticks_per_rev);
            m_pidController.SetFeedbackDevice(m_encoder);
            ret_val = true;
            break;

        case EXTERNAL_ENCODER:  
            // create encoder using A/B, set duty cycle mode
            break;

        case EXTERNAL_POTENTIOMETER:
            // create rpot
            break;

        default:
            ret_val = false;
    }

    return(ret_val);
}

bool RSpeedControllerSparkMaxCan::InvertMotor(bool invert)
{
    m_controller.SetInverted(invert);
    return(true);
}

void RSpeedControllerSparkMaxCan::SetBrakeMode(bool brake_mode)
{
    if (brake_mode)
    {
        m_controller.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else
    {
        m_controller.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

bool RSpeedControllerSparkMaxCan::SetControlMode(ControlModeType type, int device_id)
{
    bool ret_val = true;
    m_mode = type;
    switch(type)
    {
        case DUTY_CYCLE:
        case POSITION:
        case VELOCITY:
            //m_controller.SetSelectedSensorPosition(m_controller.GetSelectedSensorPosition(0),0,kTimeout);
            break;

        case FOLLOWER:
            break;

        default:
            m_mode = DUTY_CYCLE;
            ret_val = false;
            break;
    }
    return(ret_val);
}

bool RSpeedControllerSparkMaxCan::SetControlMode(ControlModeType type, rev::CANSparkMax * toFollow)
{
    bool ret_val = true;
    m_mode = type;
    switch(type)
    {
        case DUTY_CYCLE:
        case POSITION:
        case VELOCITY:
            //m_controller.SetSelectedSensorPosition(m_controller.GetSelectedSensorPosition(0),0,kTimeout);
            break;

        case FOLLOWER:
            m_controller.Follow(*toFollow);
            break;

        default:
            m_mode = DUTY_CYCLE;
            ret_val = false;
            break;
    }
    return(ret_val);
}

void RSpeedControllerSparkMaxCan::InvertSensor(bool invert, bool is_drv)
{
    if(false == is_drv)
    {
        m_sensor_invert = 1.0;
        m_controller.SetInverted(invert);
    }
    else
    {
        m_sensor_invert = invert ? -1.0 : 1.0 ;
    }
}

// Change: was +/-12V, now +/-1.0 (percent)
void RSpeedControllerSparkMaxCan::SetClosedLoopOutputLimits(float fwd_nom, float rev_nom, float fwd_peak, float rev_peak)
{
    float fwd_peak_limit = RobotUtil::limit(-1, 1, fwd_peak);
    float rev_peak_limit = RobotUtil::limit(-1, 1, rev_peak);
    m_pidController.SetOutputRange(rev_peak_limit, fwd_peak_limit);
}

void RSpeedControllerSparkMaxCan::Set(double val)  // can be duty cycle, position or velocity or device to follow
{
    m_controller.Set(val);  // is this necessary?
    if(m_mode == ControlModeType::VELOCITY)
    {
        m_pidController.SetReference(val/m_output_per_count,rev::ControlType::kVelocity);  //  TODO fix when units better understood
    }
    else if (m_mode == ControlModeType::POSITION) 
    {
        Advisory::pinfo("Setting position = %f (%f)", val, (val-m_output_offset)/m_output_per_count);
        m_pidController.SetReference((val-m_output_offset)/m_output_per_count,rev::ControlType::kPosition);
    }
    else
    {
        m_pidController.SetReference(val, rev::ControlType::kDutyCycle);
    }
}

// Return The current set speed. Value is between -1.0 and 1.0.
double RSpeedControllerSparkMaxCan::Get() { return  m_controller.Get(); }

void RSpeedControllerSparkMaxCan::SetPosition(double pos) { m_encoder.SetPosition(((pos-m_output_offset)/m_output_per_count)); }
double RSpeedControllerSparkMaxCan::GetPosition() { return ( ((m_encoder.GetPosition() * m_output_per_count) + m_output_offset) ); }
int32_t RSpeedControllerSparkMaxCan::GetRawPosition(void) { return m_encoder.GetPosition(); }

double RSpeedControllerSparkMaxCan::GetSpeed() { return (m_encoder.GetVelocity() * m_sensor_invert * m_output_per_count); }
int32_t RSpeedControllerSparkMaxCan::GetRawSpeed() { return m_encoder.GetVelocity(); }

int RSpeedControllerSparkMaxCan::GetDeviceID() { return (m_device_id); }

void  RSpeedControllerSparkMaxCan::SetCurrentLimit(uint32_t amps, uint32_t peak, uint32_t duration ) {}
void  RSpeedControllerSparkMaxCan::SetCurrentLimitEnabled(bool enabled) {}
void RSpeedControllerSparkMaxCan::SetForwardLimitSwitch (bool normally_open, bool enabled, bool zero_position) {}
void RSpeedControllerSparkMaxCan::SetReverseLimitSwitch  (bool normally_open, bool enabled, bool zero_position) {}

double RSpeedControllerSparkMaxCan::GetBusVoltage() { return 0.0; }
double RSpeedControllerSparkMaxCan::GetOutputVoltage() { return 0.0; }
double RSpeedControllerSparkMaxCan::GetMotorOutputPercent() { return 0.0; }
double RSpeedControllerSparkMaxCan::GetOutputCurrent() { return 0.0; }
double RSpeedControllerSparkMaxCan::GetActiveTrajectoryPosition() { return 0.0; }
double RSpeedControllerSparkMaxCan::GetActiveTrajectoryVelocity() { return 0.0; }

void RSpeedControllerSparkMaxCan::SetP(double p) { m_pidController.SetP(p,0); }
void RSpeedControllerSparkMaxCan::SetI(double i) { m_pidController.SetI(i,0); }
void RSpeedControllerSparkMaxCan::SetD(double d) { m_pidController.SetD(d,0); }
void RSpeedControllerSparkMaxCan::SetF(double f) { m_pidController.SetFF(f,0); }
void RSpeedControllerSparkMaxCan::SetIZone(double i_zone) { m_pidController.SetIZone(i_zone, 0); }
void RSpeedControllerSparkMaxCan::SetCruiseVelocity(double vel) {}
void RSpeedControllerSparkMaxCan::SetAcceleration(double accel) {}

double RSpeedControllerSparkMaxCan::GetP() { return  m_pidController.GetP(); }
double RSpeedControllerSparkMaxCan::GetI() { return m_pidController.GetI(); }
double RSpeedControllerSparkMaxCan::GetD() { return  m_pidController.GetD(); }
double RSpeedControllerSparkMaxCan::GetF() { return m_pidController.GetFF(); }

void RSpeedControllerSparkMaxCan::shuffleboardPublish(std::string tab, std::string motor_name) { /*frc::Shuffleboard::GetTab(tab).Add(motor_name, m_controller);*/ }
