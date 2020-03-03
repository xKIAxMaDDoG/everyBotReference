/*******************************************************************************
 *
 * File: RDigitalInput.h -- Digital Input for Simple Switches
 * 
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "frc/PWMSpeedController.h"
#include "frc/PowerDistributionPanel.h"
#include "RobonautsLibrary/RSpeedController.h"

/*******************************************************************************
 *
 * This subclass is for PWM controllers
 *
 ******************************************************************************/
class RSpeedControllerPwm : public RSpeedController
{
    public:
        RSpeedControllerPwm(frc::PWMSpeedController *sc);
        ~RSpeedControllerPwm();

        void ProcessXML(tinyxml2::XMLElement *xml) ;

        bool SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a=0, int port_b=0);
        bool InvertMotor(bool invert);

        bool SetControlMode(ControlModeType type, int deviceID=-1);
        bool SetControlMode(ControlModeType type, BaseMotorController * toFollow) { return(false); };
        bool SetControlMode(ControlModeType type, rev::CANSparkMax * toFollow)  { return(false); };

        void InvertSensor(bool invert, bool is_drv=false);

        void Set(double val);  // can be duty cycle, position or velocity or device to follow
        double Get();  // can be duty cycle, position or velocity or device to follow

        void SetPosition(double pos);
        double GetPosition();
        int32_t GetRawPosition(void);

        double GetSpeed();
        int32_t GetRawSpeed(void);

        int GetDeviceID();

        void SetCurrentLimit(uint32_t amps, uint32_t peak=0, uint32_t duration=0 );
        void SetCurrentLimitEnabled(bool enabled);
        void SetForwardLimitSwitch (bool normally_open, bool enabled = true, bool zero_position = false);
        void SetReverseLimitSwitch  (bool normally_open, bool enabled = true, bool zero_position = false);

        double GetBusVoltage();
        double GetOutputVoltage();
        double GetMotorOutputPercent();
        double GetOutputCurrent();
        double GetActiveTrajectoryPosition();
        double GetActiveTrajectoryVelocity();

        void SetP(double p);
        void SetI(double i);
        void SetD(double d);
        void SetF(double f);
        void SetIZone(double i_zone);
        void SetCruiseVelocity(double vel);
        void SetAcceleration(double accel);

        double GetP();
        double GetI();
        double GetD();
        double GetF();

        void SetFuse(int fuse);

        ctre::phoenix::motorcontrol::can::BaseMotorController* GetMotorControllerCtre() { return 0; };
        rev::CANSparkMax * GetMotorControllerSparkMax() { return 0; };

        ControllerType GetSpeedControllerType() { return(VICTOR_SPX); };


    private:
        frc::PWMSpeedController *m_controller;
        frc::PowerDistributionPanel m_power_panel;
        int m_fuse;

        uint32_t m_current_limit;
        bool m_current_limit_enabled;
};

