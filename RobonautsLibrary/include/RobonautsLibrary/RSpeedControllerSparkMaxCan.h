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

#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"
#include "RobonautsLibrary/RSpeedController.h"

/*******************************************************************************
 *
 * This subclass is for SparkMaxCAN controllers
 *
 ******************************************************************************/
class RSpeedControllerSparkMaxCan : public RSpeedController
{

    public:
        RSpeedControllerSparkMaxCan(int port_device);
        ~RSpeedControllerSparkMaxCan();

        void ProcessXML(tinyxml2::XMLElement *xml) ;

        bool SetFeedbackDevice(FeedbackDeviceType sensor_type, int port_a=0, int port_b=0);
        bool InvertMotor(bool invert);
        void SetBrakeMode(bool brake_mode);

        bool SetControlMode(ControlModeType type, int deviceID=-1);
        bool SetControlMode(ControlModeType type, BaseMotorController * toFollow)  { return(false); } ;
        bool SetControlMode(ControlModeType type, rev::CANSparkMax * toFollow);

        void InvertSensor(bool invert, bool is_drv=false);

        void SetClosedLoopOutputLimits(float min_nom, float max_nom, float min_peak, float max_peak);

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

        void shuffleboardPublish(std::string tab, std::string controller_name);
        rev::CANSparkMax * GetMotorControllerSparkMax() { return &m_controller; };
        ctre::phoenix::motorcontrol::can::BaseMotorController * GetMotorControllerCtre() { return 0; };

        ControllerType GetSpeedControllerType() { return(SPARK_MAX); };

    private:
        // basic, always used
        // changing to pointers as initialization of encoder varies based on internal vs alternate encoder
        rev::CANSparkMax m_controller;
        int m_device_id;
        rev::CANPIDController m_pidController;
        rev::CANEncoder m_encoder;

};
