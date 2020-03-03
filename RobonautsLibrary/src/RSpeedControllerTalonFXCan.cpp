/*******************************************************************************
 *
 * File: RSpeedControllerTalonFXCan.cpp
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
#include "frc/smartdashboard/SmartDashboard.h"
#include "RobonautsLibrary/RSpeedControllerTalonFXCan.h"
#include "gsu/Advisory.h"

/******************************************************************************
 *
 * Create an interface of RSpeedControllerTalonFXCan, get an APS plugged into the specified channel.
 *
 * @param   channel the channel or port that the pot is in
 *
 ******************************************************************************/
RSpeedControllerTalonFXCan::RSpeedControllerTalonFXCan(int port_device, int controlPeriodMs) :
    RSpeedController(port_device),
    m_controller(port_device)
{
    m_controller.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, controlPeriodMs, kTimeout);
}

RSpeedControllerTalonFXCan::~RSpeedControllerTalonFXCan() {}

void RSpeedControllerTalonFXCan::ProcessXML(tinyxml2::XMLElement *xml) {
    bool invert_motor = false;
    bool invert_sensor = false;
    bool brake_mode = false;

    int current_limit = 0;
    int peak_current = 0;
    int peak_current_dur = 0;

    float peak_forward_voltage = 12.0;
    float peak_reverse_voltage = -12.0;
    float nominal_forward_voltage = 0.0;
    float nominal_reverse_voltage = 0.0;

    double scale=1.0;

    xml->QueryBoolAttribute("invert", &invert_motor);
    InvertMotor(invert_motor);

    xml->QueryBoolAttribute("brake_mode", &brake_mode);
    SetBrakeMode(brake_mode);

    xml->QueryIntAttribute("current_limit", &current_limit);
    xml->QueryIntAttribute("peak_current", &peak_current);
    xml->QueryIntAttribute("peak_current_dur", &peak_current_dur);
    if (current_limit != 0 || peak_current != 0 || peak_current_dur != 0)
    {
        SetCurrentLimit(current_limit, peak_current, peak_current_dur);
    }

    xml->QueryIntAttribute("stator_current_limit", &current_limit);
    xml->QueryIntAttribute("stator_peak_current", &peak_current);
    xml->QueryIntAttribute("stator_peak_current_dur", &peak_current_dur);
    if (current_limit != 0 || peak_current != 0 || peak_current_dur != 0)
    {
        SetStatorCurrentLimit(current_limit, peak_current, peak_current_dur);
    }

    xml->QueryFloatAttribute("peak_forward_voltage", &peak_forward_voltage);
    xml->QueryFloatAttribute("peak_reverse_voltage", &peak_reverse_voltage);
    xml->QueryFloatAttribute("nominal_forward_voltage", &nominal_forward_voltage);
    xml->QueryFloatAttribute("nominal_reverse_voltage", &nominal_reverse_voltage);

    SetClosedLoopOutputLimits(nominal_forward_voltage, nominal_reverse_voltage,
            peak_forward_voltage, peak_reverse_voltage);

    tinyxml2::XMLElement *elem;
    elem = xml->FirstChildElement("encoder");
    if (elem != nullptr)
    {
        SetFeedbackDevice(RSpeedController :: INTERNAL_ENCODER);

        elem->QueryDoubleAttribute("scale", &scale);
        SensorOutputPerCount(scale);
        bool is_drive=false;

        elem->QueryBoolAttribute("drive", &is_drive);

        elem->QueryBoolAttribute("invert", &invert_sensor);
        InvertSensor(invert_sensor, is_drive);
    }
    else
    {
        elem = xml->FirstChildElement("pot");
        if (elem != nullptr)
        {
            bool sensor_wrap = false;
            elem->QueryBoolAttribute("sensor_wrap", &sensor_wrap);

            SetFeedbackDevice(RSpeedController :: INTERNAL_POTENTIOMETER, (sensor_wrap?1:0));

            elem->QueryBoolAttribute("invert", &invert_sensor);
            InvertSensor(invert_sensor);

            float raw1 	= 0.0;
            float raw2  = 5.0;
            float calc1 = 0.0;
            float calc2 = 5.0;

            float scale = 1.0;
            float offset = 0.0;

            elem->QueryFloatAttribute("p1_raw", &raw1);
            elem->QueryFloatAttribute("p2_raw", &raw2);
            elem->QueryFloatAttribute("p1_cal", &calc1);
            elem->QueryFloatAttribute("p2_cal", &calc2);

            scale	= (calc2 - calc1)/(raw2 - raw1);
            offset 	= calc1-raw1*scale;
            SensorOutputPerCount(scale, offset);

            Advisory::pinfo("  motor controller set to internal pot, sensor_wrap=%s, invert=%s, p1(%f,%f) p2(%f,%f), scale=%f offset=%f",
                    sensor_wrap?"true":"false", invert_sensor?"true":"false", raw1, calc1, raw2, calc2, scale, offset);
        }
    }

    bool has_upper_limit = false;
    bool has_lower_limit = false;
    bool upper_normally_open=false;
    bool lower_normally_open=false;
    bool upper_reset_zero=false;
    bool lower_reset_zero=false;

    elem = xml->FirstChildElement("digital_input");
    while (elem != nullptr)
    {
        const char *sw_name    = elem->Attribute("name");
        if (strcmp("upper_limit", sw_name) == 0)
        {
            elem->QueryBoolAttribute("normally_open", &upper_normally_open);
            elem->QueryBoolAttribute("reset_zero", &upper_reset_zero);
            SetForwardLimitSwitch(upper_normally_open, true, upper_reset_zero);

            has_upper_limit = true;
        }
        else if (strcmp("lower_limit", sw_name) == 0)
        {
            elem->QueryBoolAttribute("normally_open", &lower_normally_open);
            elem->QueryBoolAttribute("reset_zero", &lower_reset_zero);
            SetReverseLimitSwitch(lower_normally_open, true, lower_reset_zero);

            has_lower_limit = true;
        }
        else
        {
            Advisory::pinfo ("motor limit switch unknown name of %s found", sw_name);
        }

        elem = elem->NextSiblingElement("digital_input");
    }

    // if the limits were set in a previous configuration,
    // we should disable them now
    if (! has_upper_limit)
    {
        SetForwardLimitSwitch(false, false, false);
    }

    if (! has_lower_limit)
    {
        SetReverseLimitSwitch(false, false, false);
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

    Advisory::pinfo("    creating CAN Talon Speed Controller on device %d, invert_m=%s, invert_s=%s scale=%.3f, kf=%f, kp=%f, ki=%f, kd=%f, cruise_velocity=%f, acceleration=%f",
            m_device_id, invert_motor?"true":"false",invert_sensor?"true":"false", scale, kf, kp, ki, kd, cruise_velocity, acceleration);
}

/******************************************************************************
 *
 * If sensor_type is INTERNAL_POTENTIOMETER:
 *     port_a==0 means no sensor wrap, port_a!=0 means sensor wrap
 *
 ******************************************************************************/
bool RSpeedControllerTalonFXCan::SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a, int port_b)
{
    bool ret_val = false;

    switch(sensor_type)
    {
        case INTERNAL_ENCODER:
            //            m_controller.SetFeedbackDevice(CANTalon::QuadEncoder);
            m_controller.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeout);
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

bool RSpeedControllerTalonFXCan::InvertMotor(bool invert)
{
    m_controller.SetInverted(invert);
    return(true);
}

void RSpeedControllerTalonFXCan::SetBrakeMode(bool brake_mode)
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

bool RSpeedControllerTalonFXCan::SetControlMode(ControlModeType type, int device_id)
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

bool RSpeedControllerTalonFXCan::SetControlMode(ControlModeType type, BaseMotorController * toFollow)
{
    bool ret_val = true;

    Advisory::pinfo("Talon FX following setControlMode");
    if(toFollow != nullptr)
    {
        Advisory::pinfo("Talon FX %d type %d SetCtrlMode", type, toFollow->GetDeviceID());
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
                Advisory::pinfo("setting talon FX %d to follow ID %d", m_controller.GetDeviceID(), toFollow->GetDeviceID());
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
        Advisory::pinfo("SetControlMode CAN toFollow a CTRE device null");
        ret_val = false;
    }

    return(ret_val);
}

BaseMotorController* RSpeedControllerTalonFXCan::GetMotorControllerCtre() 
{ 
    return &m_controller; 
}


void RSpeedControllerTalonFXCan::InvertSensor(bool invert, bool is_drv)
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

// Change: was +/-12V, now +/-1.0 (percent)
void RSpeedControllerTalonFXCan::SetClosedLoopOutputLimits(float fwd_nom, float rev_nom, float fwd_peak, float rev_peak)
{
    m_controller.ConfigPeakOutputForward(fwd_peak, kTimeout);
    m_controller.ConfigPeakOutputReverse(rev_peak, kTimeout);
    m_controller.ConfigNominalOutputForward(fwd_nom, kTimeout);
    m_controller.ConfigNominalOutputReverse(rev_nom, kTimeout);
}

void RSpeedControllerTalonFXCan::Set(double val)  // can be duty cycle, position or velocity or device to follow
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
double RSpeedControllerTalonFXCan::Get() { return  0.0; }

void RSpeedControllerTalonFXCan::SetPosition(double pos) { m_controller.SetSelectedSensorPosition(((pos-m_output_offset)/m_output_per_count), 0, kTimeout); }
double RSpeedControllerTalonFXCan::GetPosition() { return ( ((m_controller.GetSelectedSensorPosition(0) * m_output_per_count) + m_output_offset) ); }
int32_t RSpeedControllerTalonFXCan::GetRawPosition(void) { return m_controller.GetSelectedSensorPosition(0); }

double RSpeedControllerTalonFXCan::GetSpeed() { return (m_controller.GetSelectedSensorVelocity(0) * m_sensor_invert * m_output_per_count / 0.1); }
int32_t RSpeedControllerTalonFXCan::GetRawSpeed() { return m_controller.GetSelectedSensorVelocity(0); }

int RSpeedControllerTalonFXCan::GetDeviceID() { return m_controller.GetDeviceID(); }

/******************************************************************************
 *
 * @param amps
 * @param peak (amps)
 * @param duration (milliseconds)
 * 
 ******************************************************************************/
void  RSpeedControllerTalonFXCan::SetCurrentLimit(uint32_t amps, uint32_t peak, uint32_t duration )
{
    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration sclc(true, amps, peak, kTimeout);
    m_controller.ConfigSupplyCurrentLimit(sclc);
}

void  RSpeedControllerTalonFXCan::SetCurrentLimitEnabled(bool enabled)
{
    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration sclc;
    m_controller.ConfigGetSupplyCurrentLimit(sclc);
    sclc.enable = enabled;
    m_controller.ConfigSupplyCurrentLimit(sclc);
}

void  RSpeedControllerTalonFXCan::SetStatorCurrentLimit(uint32_t amps, uint32_t peak, uint32_t duration )
{
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration sclc(true, amps, peak, kTimeout);
    m_controller.ConfigStatorCurrentLimit(sclc);
}

void  RSpeedControllerTalonFXCan::SetStatorCurrentLimitEnabled(bool enabled)
{
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration sclc;
    m_controller.ConfigGetStatorCurrentLimit(sclc);
    sclc.enable = enabled;
    m_controller.ConfigStatorCurrentLimit(sclc);
}
/******************************************************************************
 *
 * Configuring the Can Limit Switch
 *
 * @param normally_open 	true if the switch is wired to be normally open,
 *                          false if it is normally closed
 *
 * @param enabled			if true (the default) it will the talon will be
 * 							configured to have a limit switch wired to the
 * 							speed controller, if false it will turn off
 * 							the limit switch
 * 							NOTE: this could be updated to also allow for
 * 							the Talon to use external limit switches.
 *
 ******************************************************************************/
void RSpeedControllerTalonFXCan::SetForwardLimitSwitch (bool normally_open, bool enabled, bool zero_position)
{
    Advisory::pinfo("        RSpeedControllerTalonFXCan::SetForwardLimitSwitch(%d,%d,%d)", normally_open,enabled,zero_position);
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
 * @param normally_open 	true if the switch is wired to be normally open,
 *                          false if it is normally closed
 *
 * @param enabled			if true (the default) it will the talon will be
 * 							configured to have a limit switch wired to the
 * 							speed controller, if false it will turn off
 * 							the limit switch
 * 							NOTE: this could be updated to also allow for
 * 							the Talon to use external limit switches.
 *
 ******************************************************************************/
void RSpeedControllerTalonFXCan::SetReverseLimitSwitch  (bool normally_open, bool enabled, bool zero_position)
{
    Advisory::pinfo("        RSpeedControllerTalonFXCan::SetReverseLimitSwitch(%d,%d,%d)", normally_open,enabled,zero_position);
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

double RSpeedControllerTalonFXCan::GetBusVoltage() { return m_controller.GetBusVoltage(); }
double RSpeedControllerTalonFXCan::GetOutputVoltage() { return m_controller.GetMotorOutputVoltage(); }
double RSpeedControllerTalonFXCan::GetMotorOutputPercent() { return m_controller.GetMotorOutputPercent(); }
double RSpeedControllerTalonFXCan::GetOutputCurrent() { return m_controller.GetOutputCurrent(); }
double RSpeedControllerTalonFXCan::GetActiveTrajectoryPosition() { return m_controller.GetActiveTrajectoryPosition(); }
double RSpeedControllerTalonFXCan::GetActiveTrajectoryVelocity() { return m_controller.GetActiveTrajectoryVelocity(); }

void RSpeedControllerTalonFXCan::SetP(double p) { m_controller.Config_kP(0, p, kTimeout); }
void RSpeedControllerTalonFXCan::SetI(double i) { m_controller.Config_kI(0, i, kTimeout); }
void RSpeedControllerTalonFXCan::SetD(double d) { m_controller.Config_kD(0, d, kTimeout); }
void RSpeedControllerTalonFXCan::SetF(double f) { m_controller.Config_kF(0, f, kTimeout); }
void RSpeedControllerTalonFXCan::SetIZone(double i_zone) { m_controller.Config_IntegralZone(0, (int)i_zone, kTimeout); }
void RSpeedControllerTalonFXCan::SetCruiseVelocity(double vel) { m_controller.ConfigMotionCruiseVelocity((int)(vel/m_output_per_count / 10.0), kTimeout); }
void RSpeedControllerTalonFXCan::SetAcceleration(double accel) { m_controller.ConfigMotionAcceleration((int)(accel/m_output_per_count / 10.0), kTimeout); }

double RSpeedControllerTalonFXCan::GetP() { return  m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_P, 0, kTimeout); }
double RSpeedControllerTalonFXCan::GetI() { return  m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_I, 0, kTimeout); }
double RSpeedControllerTalonFXCan::GetD() { return  m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_D, 0, kTimeout); }
double RSpeedControllerTalonFXCan::GetF() { return  m_controller.ConfigGetParameter(ParamEnum::eProfileParamSlot_F, 0, kTimeout); }

bool RSpeedControllerTalonFXCan::isForwardLimitPressed(void)
{
    ctre::phoenix::motorcontrol::TalonFXSensorCollection &sensor_data = m_controller.GetSensorCollection();
    return (sensor_data.IsFwdLimitSwitchClosed() > 0);
}

bool RSpeedControllerTalonFXCan::isReverseLimitPressed(void)
{
    ctre::phoenix::motorcontrol::TalonFXSensorCollection &sensor_data = m_controller.GetSensorCollection();
    return (sensor_data.IsRevLimitSwitchClosed() > 0);
}

void RSpeedControllerTalonFXCan::publish(std::string tab, std::string motor_name) {
    static bool inited = false;
    static nt::NetworkTableEntry nt_mode;
    static nt::NetworkTableEntry nt_raw_position;
    static nt::NetworkTableEntry nt_actual_position;
    static nt::NetworkTableEntry nt_raw_velocity;
    static nt::NetworkTableEntry nt_actual_velocity;
    static nt::NetworkTableEntry nt_forward_limit_switch;
    static nt::NetworkTableEntry nt_reverse_limit_switch;

    if ( !inited ) {
        static nt::NetworkTableEntry nt_p;
        static nt::NetworkTableEntry nt_i;
        static nt::NetworkTableEntry nt_d;
        static nt::NetworkTableEntry nt_f;
        static nt::NetworkTableEntry nt_output_per_count;
        static nt::NetworkTableEntry nt_output_offset;
        nt_p = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/p", 0).GetEntry();
        nt_i = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/i", 0).GetEntry();
        nt_d = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/d", 0).GetEntry();
        nt_f = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/f", 0).GetEntry();
        nt_output_per_count = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/output_per_count", 0).GetEntry();
        nt_output_offset = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/config/output_offset", 0).GetEntry();
        nt_p.SetDouble(GetP());
        nt_i.SetDouble(GetI());
        nt_d.SetDouble(GetD());
        nt_f.SetDouble(GetF());
        nt_output_per_count.SetDouble(m_output_per_count);
        nt_output_offset.SetDouble(m_output_offset);

        frc::Shuffleboard::GetTab(tab).Add(motor_name+"/type", "TalonFX");
        nt_mode = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/mode", 0).GetEntry();
        nt_raw_position= frc::Shuffleboard::GetTab(tab).Add(motor_name+"/raw_position", 0).GetEntry();
        nt_raw_velocity= frc::Shuffleboard::GetTab(tab).Add(motor_name+"/raw_velocity", 0).GetEntry();
        nt_actual_position= frc::Shuffleboard::GetTab(tab).Add(motor_name+"/actual_position", 0).GetEntry();
        nt_actual_velocity= frc::Shuffleboard::GetTab(tab).Add(motor_name+"/actual_velocity", 0).GetEntry();
        nt_forward_limit_switch = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/forward_limit_switch", 0).GetEntry();
        nt_reverse_limit_switch = frc::Shuffleboard::GetTab(tab).Add(motor_name+"/reverse_limit_switch", 0).GetEntry();

        inited = true;
    }

    nt_mode.SetDouble(m_mode);
    nt_raw_position.SetDouble(GetRawPosition());
    nt_raw_velocity.SetDouble(GetRawSpeed());
    nt_actual_position.SetDouble(GetPosition());
    nt_actual_velocity.SetDouble(GetSpeed());   
    nt_forward_limit_switch.SetBoolean(isForwardLimitPressed());
    nt_reverse_limit_switch.SetBoolean(isReverseLimitPressed());
}
