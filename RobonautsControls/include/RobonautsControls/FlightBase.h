/*******************************************************************************
 *
 * File: FlightBase.h
 * 
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "frc/smartdashboard/SendableChooser.h"
#include "frc/TimedRobot.h"            

#include "RobonautsLibrary/OIObserver.h"
#include "RobonautsLibrary/MacroController.h"
#include "RobonautsControls/ControlThread.h"

/*******************************************************************************
 *
 * The main class for this robot
 * 
 ******************************************************************************/
class FlightBase : public frc::TimedRobot, public OIObserver
{
    public:
        enum Phase
        {
            INIT, DISABLED, AUTON, TELEOP, TEST
        };

        // Operator Interface Input Commands
        enum Command
        {
            CMD_START_MACRO = 200, CMD_START_MACRO_MAX = 299,
            CMD_ABORT_MACRO = 300, CMD_ABORT_MACRO_MAX = 399
        };

        FlightBase(void);
        ~FlightBase();

        void RobotInit(void);
        void LoadRobot(std::string file);
        void UpdateAutonConfig(void);

        void DisabledInit(void);
        void DisabledPeriodic(void);

        void AutonomousInit(void);
        void AutonomousPeriodic(void);

        void TeleopInit(void);
        void TeleopPeriodic(void);

        void TestInit(void);
        void TestPeriodic(void);

        void setAnalog(int id, float val);
        void setDigital(int id, bool val);
        void setInt(int id, int val);

    private:
        class AutonIndex
        {
            public:
                AutonIndex(int i) : index(i) {}
                int value() {return index;}
            private:
                int index;
        };

        bool isAutonAvailable();
        void publish(void);

        std::vector<ControlThread *> robot_controls;

        MacroController *m_macro_controller;
        std::vector<std::string> m_macro_name_list;
        std::vector<std::string> m_macro_file_list;

        std::vector<std::string> m_auton_name_list;
        std::vector<std::string> m_auton_file_list;

        int         m_auton_file_count;
        int   		m_auton_select;
        int 		m_auton_select_loaded;
        int		m_auton_select_mismatch;
        bool  		m_auton_enabled;
        float 		m_auton_delay;
        bool  		m_auton_delay_enabled;
        double		m_auton_start_time;
        bool 		m_auton_started;
        std::string m_auton_description;
        frc::SendableChooser<AutonIndex *> m_auton_chooser;

        uint16_t 	m_publish_count;

        Phase m_robot_phase;
};
