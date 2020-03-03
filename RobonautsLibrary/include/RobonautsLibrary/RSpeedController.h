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

#include <string>
#include <cstdint>
#include "gsu/tinyxml2.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

/*******************************************************************************
 *
 * This class provides the definition of an interface for speed controllers,
 * providing a common interface for both PWM speed controller and CAN Talon
 * speed controllers.
 * 
 ******************************************************************************/
class RSpeedController
{
    public:
        static const int kTimeout=0;

        enum FeedbackDeviceType
        {
            EXTERNAL_ENCODER=0,
            INTERNAL_ENCODER,
            EXTERNAL_POTENTIOMETER,
            INTERNAL_POTENTIOMETER,
            INTERNAL_BRUSHLESS_HALL,
        };

        enum ControlModeType
        {
            DUTY_CYCLE=0,
            POSITION,
            VELOCITY,
            TRAPEZOID,
            FOLLOWER,
            UNKNOWN_TYPE
        };
        enum ControllerType
        {
            TALON_SRX = 0,
            VICTOR_SPX,
            TALON_FX,
            SPARK_MAX,
            PWM
        };

        virtual ~RSpeedController(void);
        virtual void ProcessXML(tinyxml2::XMLElement *xml) = 0;

        // port a is sometimes re-purposed; 
        //      for TalonSRX, with potentiometer, it indicates continuous or not
        //      for SparkMax with alternate encode, it indicates quad counts/rev (aka 4x ticks/rev)
        virtual bool SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a=0, int port_b=0) = 0;
        virtual bool InvertMotor(bool invert) = 0;
        virtual void SetBrakeMode(bool brake_mode) {}

        virtual bool SetControlMode(ControlModeType type, int deviceID=-1) = 0;
        virtual bool SetControlMode(ControlModeType type, BaseMotorController *toFollow) = 0;
        virtual bool SetControlMode(ControlModeType type, rev::CANSparkMax * toFollow) = 0;

        virtual ControlModeType GetControlMode();

        virtual void InvertSensor(bool invert, bool is_drive=false) = 0;
        virtual void SensorOutputPerCount(double conversion, double offset=0);

        virtual void SetClosedLoopOutputLimits(float fwd_nom, float rev_nom, float fwd_peak, float rev_peak) {}

        virtual void Set(double val) = 0;  // can be duty cycle, position or velocity or device to follow
        virtual double Get() = 0;  // can be duty cycle, position or velocity or device to follow

        virtual void SetPosition(double pos)= 0;
        virtual double GetPosition() = 0;
        virtual int32_t GetRawPosition(void) = 0;

        virtual double GetSpeed() = 0;
        virtual int32_t GetRawSpeed(void) = 0;

        virtual int GetDeviceID() = 0;

        virtual void SetCurrentLimit(uint32_t amps, uint32_t peak=0, uint32_t duration=0 ) = 0;
        virtual void SetCurrentLimitEnabled(bool enabled) = 0;
        virtual void SetForwardLimitSwitch (bool normally_open, bool enabled = true, bool zero_position = false) = 0;
        virtual void SetReverseLimitSwitch  (bool normally_open, bool enabled = true, bool zero_position = false) = 0;

        virtual double GetBusVoltage() = 0;
        virtual double GetOutputVoltage() = 0;
        virtual double GetMotorOutputPercent() = 0;
        virtual double GetOutputCurrent() = 0;
        virtual double GetActiveTrajectoryPosition() = 0;
        virtual double GetActiveTrajectoryVelocity() = 0;

        virtual void SetP(double p) = 0;
        virtual void SetI(double i) = 0;
        virtual void SetD(double d) = 0;
        virtual void SetF(double f) = 0;
        virtual void SetIZone(double i_zone) = 0;
        virtual void SetCruiseVelocity(double vel) = 0;
        virtual void SetAcceleration(double accel) = 0;

        virtual double GetP() = 0;
        virtual double GetI() = 0;
        virtual double GetD() = 0;
        virtual double GetF() = 0;

        virtual bool isForwardLimitPressed(void);
        virtual bool isReverseLimitPressed(void);

        virtual void publish(std::string tab, std::string controller_name) {}
        virtual ctre::phoenix::motorcontrol::can::BaseMotorController * GetMotorControllerCtre() { return 0; }
        virtual rev::CANSparkMax * GetMotorControllerSparkMax() { return 0; }

        virtual ControllerType GetSpeedControllerType() = 0;

    protected:
        RSpeedController(int device_id = -1);

        int m_device_id;
        ControlModeType m_mode;
        double m_sensor_invert;
        double m_output_per_count;
        double m_output_offset;
};

