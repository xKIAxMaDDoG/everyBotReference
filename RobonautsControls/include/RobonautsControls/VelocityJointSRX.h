/*******************************************************************************
 *
 * File: VelocityJointSRX.h
 * 
 *
 * 
 * Written by:
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "RobonautsLibrary/RSpeedController.h"
#include "RobonautsLibrary/RPot.h"
#include "RobonautsLibrary/RAbsPosSensor.h"
#include "RobonautsLibrary/SimplePID.h"
#include "RobonautsLibrary/RobotUtil.h"
#include "RobonautsLibrary/DataLogger.h"
#include "RobonautsLibrary/OIDriverStation.h"

#include "RobonautsControls/PeriodicControl.h"
#include "RobonautsLibrary/MacroStep.h"
#include "RobonautsLibrary/OIObserver.h"
#include "RobonautsLibrary/OIController.h"

/*******************************************************************************    
 * 
 * Create an instance of a vjs control and connect it to the specified
 * servo and inputs
 * 
 * This class is designed to be created from an XML element with the following
 * format, portions contained in [ ] are optional.
 * 
 *  <control type="vjs" [name="unnamed"] [closed_loop="false"] [period="0.1"]
 *          [setpoint0="0.0"] [setpoint1="0.0"] [setpoint2="0.0"] [setpoint3="0.0"] >
 *
 *      [
 *        <pid [kp="kp"] [ki="ki"] [kd="kd"]
 *           [targ_min="targ_min"] [targ_max="targ_max"] [targ_thp="targ_thp"]
 *           [cntl_min="cntl_min"] [cntl_max="cntl_max"] />
 *      ]
 *
 *      [<oi name="closed_loop_state"  device="switches" chan="1" [invert="false"]/>]
 *
 *      [<oi name="analog"    device="pilot" chan="1" [scale="1.0"|invert="false"]/>]
 *      [<oi name="increment" step="0.1" device="pilot" chan="5" [invert="false"]/>]
 *      [<oi name="decrement" step="0.1" device="pilot" chan="7" [invert="false"]/>]
 *      [<oi name="stop"      device="pilot" chan="1" [invert="false"]/>]
 *
 *      [<oi name="setpoint_idx" device="pilot" chan="0" [scale="0.2222222"]/>
 *
 *      [<oi name="setpoint0" device="pilot" chan="2" [invert="false"]/>]
 *      [<oi name="setpoint1" device="pilot" chan="3" [invert="false"]/>]
 *      [<oi name="setpoint2" device="pilot" chan="3" [invert="false"]/>]
 *      [<oi name="setpoint3" device="pilot" chan="3" [invert="false"]/>]
 *
 *  </control>
 *  
 *
 ******************************************************************************/
class VelocityJointSRX : public PeriodicControl, public OIObserver
{
    public:
        enum Command
        {
            CMD_CLOSED_LOOP_STATE = 0,
            CMD_MOVE_ANALOG,
            CMD_SETPOINT_0,
            CMD_SETPOINT_1,
            CMD_SETPOINT_2,
            CMD_SETPOINT_3,
            CMD_SETPOINT_IDX,
            CMD_INCREMENT_POS,
            CMD_DECREMENT_POS,
            CMD_ALLIANCE_COLOR,
			CMD_UPDATE_SETPOINT
        };

        static const uint8_t NUM_SETPOINTS = 4;

        VelocityJointSRX(std::string control_name, tinyxml2::XMLElement *xml);
        ~VelocityJointSRX(void);

          void controlInit(void);
        void updateConfig(void);

        void disabledInit();
        void autonomousInit();
        void teleopInit();
        void testInit();
        void doPeriodic();

        void publish(void);

        void setAnalog(int id, float val);
        void setDigital(int id, bool val);
        void setInt(int id, int val);

        void setPosition(float val);
        float getPosition(void);
        float getTargetPosition(void);

        bool isClosedLoop(void);
        
    private:        
        void applySetpoint(bool on, int idx);

        void initLogFile(std::string phase);
        void logVariables(void);

        RSpeedController *m_motor;

        SimplePID         *m_pid;

        float m_increment_step;
        float m_decrement_step;
        float m_delta_position;

        bool m_is_ready;

        float m_raw_position;     // logged
        float m_actual_position;  // logged

        float m_target_power;  // logged
        float m_target_position;  // logged
        float m_command_power;   // logged
        float m_max_power_delta;

        float m_setpoint[NUM_SETPOINTS];
        uint8_t m_setpoint_index;  // logged
        uint8_t m_num_setpoints;

        float m_position_adjust_rate;

        bool m_closed_loop;  // logged

        // create data logger for drive system
        DataLogger *m_vjs_auton_log;
        DataLogger *m_vjs_teleop_log;
        DataLogger *m_vjs_log;

};

/*******************************************************************************
 *
 *
 ******************************************************************************/
class MSVelJointSRXSetPosition : public MacroStepSequence
{
    public:
        MSVelJointSRXSetPosition(std::string type, tinyxml2::XMLElement *xml, void *control);

        void init(void);
        MacroStep * update(void);

    private:
        VelocityJointSRX *m_parent_control;
        float m_position;
};

/*******************************************************************************
 *
 *
 ******************************************************************************/
class MSVelJointSRXIsReady : public MacroStepCondition
{
    public:
        MSVelJointSRXIsReady(std::string type, tinyxml2::XMLElement *xml, void *control);

        void init(void);
        MacroStep * update(void);

    private:
        VelocityJointSRX *m_parent_control;
        float m_position_tolerance;
};
