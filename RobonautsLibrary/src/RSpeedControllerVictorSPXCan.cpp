/*******************************************************************************
 *
 * File: RAbsPosSensor.cpp
 * 
 * This file contains the definition of a class for interacting with an
 * Absolute Position Sensor.
 *
 * Written by:
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/
#include "RobonautsLibrary/RSpeedControllerVictorSpxCan.h"
#include "gsu/Advisory.h"

/******************************************************************************
 *
 * Create an interface of RSpeedControllerCan, get an APS plugged into the specified channel.
 *
 * @param   channel the channel or port that the pot is in
 *
 ******************************************************************************/
RSpeedControllerVictorSpxCan::RSpeedControllerVictorSpxCan(int port_device, int controlPeriodMs)
    : RSpeedController(port_device)
    , m_controller(port_device)
{
    m_controller.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, controlPeriodMs, kTimeout);
}

RSpeedControllerVictorSpxCan::~RSpeedControllerVictorSpxCan() {}

void RSpeedControllerVictorSpxCan::ProcessXML(tinyxml2::XMLElement *xml) {
    bool invert_motor = false;
    bool brake_mode = false;

    xml->QueryBoolAttribute("invert", &invert_motor);
    InvertMotor(invert_motor);

    xml->QueryBoolAttribute("brake_mode", &brake_mode);
    SetBrakeMode(brake_mode);
}

/******************************************************************************
 *
 * If sensor_type is INTERNAL_POTENTIOMETER:
 *     port_a==0 means no sensor wrap, port_a!=0 means sensor wrap
 *
 ******************************************************************************/
bool RSpeedControllerVictorSpxCan::SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a, int port_b)
{
    bool ret_val = false;

    switch(sensor_type)
    {
        case INTERNAL_ENCODER:
            m_controller.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeout);
            ret_val = true;
            break;

        case INTERNAL_POTENTIOMETER:
            m_controller.ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, kTimeout);
            m_controller.ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, (port_a==0), 0x00, 0x00, 0x00);
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

bool RSpeedControllerVictorSpxCan::InvertMotor(bool invert)
{
    m_controller.SetInverted(invert);
    return(true);
}

void RSpeedControllerVictorSpxCan::SetBrakeMode(bool brake_mode)
{
    if (brake_mode)
    {
        m_controller.SetNeutralMode(NeutralMode::Brake);
    }
    else
    {
        m_controller.SetNeutralMode(NeutralMode::Coast);
    }
}

bool RSpeedControllerVictorSpxCan::SetControlMode(ControlModeType type, int device_id)
{
    bool ret_val = true;

    m_mode = type;
    switch(type)
    {
        case DUTY_CYCLE:
        case POSITION:
        case VELOCITY:
        case TRAPEZOID:
            m_controller.SetSelectedSensorPosition(m_controller.GetSelectedSensorPosition(0),0,kTimeout);
            break;

        case FOLLOWER:
            m_controller.Set(ControlMode::Follower, device_id);
            break;

        default:
            m_mode = DUTY_CYCLE;
            ret_val = false;
            break;
    }

    return(ret_val);
}

bool RSpeedControllerVictorSpxCan::SetControlMode(ControlModeType type, BaseMotorController *toFollow)
{
    bool ret_val = true;
    if(toFollow != nullptr)
    {
        m_mode = type;
        switch(type)
        {
            case DUTY_CYCLE:
            case POSITION:
            case VELOCITY:
            case TRAPEZOID:
                m_controller.SetSelectedSensorPosition(m_controller.GetSelectedSensorPosition(0),0,kTimeout);
                break;

            case FOLLOWER:
                Advisory::pinfo("setting victor SPX %d to follow ID %d", m_controller.GetDeviceID(), toFollow->GetDeviceID());
                m_controller.Follow(*toFollow);
                break;

            default:
                m_mode = DUTY_CYCLE;
                ret_val = false;
                break;
        }
    }
    else
    {
        Advisory::pinfo("SetControlMode CAN toFollow TalonFX null");
        ret_val = false;
    }

    return(ret_val);
}


// Change: was +/-12V, now +/-1.0 (percent)
void RSpeedControllerVictorSpxCan::SetClosedLoopOutputLimits(float fwd_nom, float rev_nom, float fwd_peak, float rev_peak)
{
    m_controller.ConfigPeakOutputForward(fwd_peak, kTimeout);
    m_controller.ConfigPeakOutputReverse(rev_peak, kTimeout);
    m_controller.ConfigNominalOutputForward(fwd_nom, kTimeout);
    m_controller.ConfigNominalOutputReverse(rev_nom, kTimeout);
}

void RSpeedControllerVictorSpxCan::InvertSensor(bool invert, bool is_drv)
{
    if(false == is_drv)
    {
        m_sensor_invert = 1.0;
        m_controller.SetSensorPhase(invert);
    }
    else
    {
        m_sensor_invert = invert ? -1.0 : 1.0 ;
    }
}

void RSpeedControllerVictorSpxCan::Set(double val)  // can be duty cycle, position or velocity or device to follow
{
    if(m_mode == ControlModeType::VELOCITY)
    {
        m_controller.Set(ControlMode::Velocity, val/m_output_per_count / 10.0);  //  TODO fix when units better understood
    }
    else if (m_mode == ControlModeType::POSITION)
    {
        m_controller.Set(ControlMode::Position, (val-m_output_offset)/m_output_per_count);
    }
    else if (m_mode == ControlModeType::TRAPEZOID)
    {
        m_controller.Set(ControlMode::MotionMagic, (val-m_output_offset)/m_output_per_count);
    }
    else
    {
        m_controller.Set(ControlMode::PercentOutput, val);  //  TODO fix when units better understood
   }
}

double RSpeedControllerVictorSpxCan::Get() { return  0.0; }

void RSpeedControllerVictorSpxCan::SetPosition(double pos) {
    m_controller.SetSelectedSensorPosition(((pos-m_output_offset)/m_output_per_count), 0, kTimeout);
}
double RSpeedControllerVictorSpxCan::GetPosition()
{
    return ( ((m_controller.GetSelectedSensorPosition(0) * m_output_per_count) + m_output_offset) );
}
int32_t RSpeedControllerVictorSpxCan::GetRawPosition(void) { return m_controller.GetSelectedSensorPosition(0); }

double RSpeedControllerVictorSpxCan::GetSpeed()
{
    return (m_controller.GetSelectedSensorVelocity(0) * m_sensor_invert * m_output_per_count / 0.1);  // todo no hard code
}
int32_t RSpeedControllerVictorSpxCan::GetRawSpeed() { return m_controller.GetSelectedSensorVelocity(0); }

int RSpeedControllerVictorSpxCan::GetDeviceID() { return m_controller.GetDeviceID(); }

void  RSpeedControllerVictorSpxCan::SetCurrentLimit(uint32_t amps, uint32_t peak, uint32_t duration ) {}
void  RSpeedControllerVictorSpxCan::SetCurrentLimitEnabled(bool enabled) {}

/******************************************************************************
 *
 * Configuring the Can Limit Switch
 *
 * @param normally_open     true if the switch is wired to be normally open,
 *                          false if it is normally closed
 *
 * @param enabled            if true (the default) it will the talon will be
 *                             configured to have a limit switch wired to the
 *                             speed controller, if false it will turn off
 *                             the limit switch
 *                             NOTE: this could be updated to also allow for
 *                             the Talon to use external limit switches.
 *
 ******************************************************************************/
void RSpeedControllerVictorSpxCan::SetForwardLimitSwitch (bool normally_open, bool enabled, bool zero_position)
{
    Advisory::pinfo("RSpeedControllerCan::SetForwardLimitSwitch(%d,%d)", normally_open,enabled);
    if (enabled)
    {
        m_controller.ConfigForwardLimitSwitchSource(
                LimitSwitchSource_FeedbackConnector,
                normally_open?LimitSwitchNormal_NormallyOpen:LimitSwitchNormal_NormallyClosed,
                kTimeout);
                
        m_controller.ConfigClearPositionOnLimitF(zero_position);
    }
    else
    {
        m_controller.ConfigForwardLimitSwitchSource(
                LimitSwitchSource_Deactivated,
                LimitSwitchNormal_Disabled,
                kTimeout);
    }
}

/******************************************************************************
 *
 * Configuring the Can Limit Switch
 *
 * @param normally_open     true if the switch is wired to be normally open,
 *                          false if it is normally closed
 *
 * @param enabled            if true (the default) it will the talon will be
 *                             configured to have a limit switch wired to the
 *                             speed controller, if false it will turn off
 *                             the limit switch
 *                             NOTE: this could be updated to also allow for
 *                             the Talon to use external limit switches.
 *
 ******************************************************************************/
void RSpeedControllerVictorSpxCan::SetReverseLimitSwitch  (bool normally_open, bool enabled, bool zero_position)
{
    if (enabled)
    {
        m_controller.ConfigReverseLimitSwitchSource(
                LimitSwitchSource_FeedbackConnector,
                normally_open?LimitSwitchNormal_NormallyOpen:LimitSwitchNormal_NormallyClosed,
                kTimeout);
                
        m_controller.ConfigClearPositionOnLimitR(zero_position);
    }
    else
    {
        m_controller.ConfigReverseLimitSwitchSource(
                LimitSwitchSource_Deactivated,
                LimitSwitchNormal_Disabled,
                kTimeout);
    }

}

double RSpeedControllerVictorSpxCan::GetBusVoltage() { return m_controller.GetBusVoltage(); }
double RSpeedControllerVictorSpxCan::GetOutputVoltage() { return m_controller.GetMotorOutputVoltage(); }
double RSpeedControllerVictorSpxCan::GetMotorOutputPercent() { return m_controller.GetMotorOutputPercent(); }
double RSpeedControllerVictorSpxCan::GetOutputCurrent() { return 0.0; }
double RSpeedControllerVictorSpxCan::GetActiveTrajectoryPosition() { return m_controller.GetActiveTrajectoryPosition(); }
double RSpeedControllerVictorSpxCan::GetActiveTrajectoryVelocity() { return m_controller.GetActiveTrajectoryVelocity(); }

void RSpeedControllerVictorSpxCan::SetP(double p) { m_controller.Config_kP(0, p, kTimeout); }
void RSpeedControllerVictorSpxCan::SetI(double i) { m_controller.Config_kI(0, i, kTimeout); }
void RSpeedControllerVictorSpxCan::SetD(double d) { m_controller.Config_kD(0, d, kTimeout); }
void RSpeedControllerVictorSpxCan::SetF(double f) { m_controller.Config_kF(0, f, kTimeout); }
void RSpeedControllerVictorSpxCan::SetIZone(double i_zone) { m_controller.Config_IntegralZone(0, (int)i_zone, kTimeout); }
void RSpeedControllerVictorSpxCan::SetCruiseVelocity(double vel) { m_controller.ConfigMotionCruiseVelocity((int)(vel/m_output_per_count / 10.0), kTimeout); }
void RSpeedControllerVictorSpxCan::SetAcceleration(double accel) { m_controller.ConfigMotionAcceleration((int)(accel/m_output_per_count / 10.0), kTimeout); }

double RSpeedControllerVictorSpxCan::GetP() { return m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_P, 0, kTimeout); }
double RSpeedControllerVictorSpxCan::GetI() { return m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_I, 0, kTimeout); }
double RSpeedControllerVictorSpxCan::GetD() { return m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_D, 0, kTimeout); }
double RSpeedControllerVictorSpxCan::GetF() { return m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_F, 0, kTimeout); }

