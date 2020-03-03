/*******************************************************************************
 *
 * File: FlightBase.cpp
 *
 * Written by: 
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/
#include <cstring>
#include <math.h>
#include <string>
#include <cstdint>

#include "frc/livewindow/LiveWindow.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "gsi/Path.h"
#include "gsi/Time.h"

#include "gsu/tinyxml2.h"
#include "gsu/Advisory.h"
#include "gsu/AdvisoryFile.h"
#include "gsu/AdvisoryStdOut.h"
#include "gsu/SegmentedFile.h"
#include "gsu/PathUtil.h"
#include "gsu/Parameter.h"
#include "gsu/ParameterFile.h"

#include "RobonautsLibrary/Macro.h"
#include "RobonautsLibrary/MacroController.h"
#include "RobonautsLibrary/DataLogger.h"
#include "RobonautsLibrary/OIController.h"

#include "RobonautsControls/ControlFactory.h"

#include "RobonautsControls/FlightBase.h"


/*******************************************************************************
 *
 ******************************************************************************/
FlightBase::FlightBase(void)
{
    m_macro_controller    = nullptr;

    m_auton_file_count = 0;
    m_auton_select = 0;
    m_auton_select_mismatch = -1;
    m_auton_select_loaded = -2;
    m_auton_enabled = true;
    m_auton_delay = 0.0;
    m_auton_delay_enabled = false;
    m_auton_start_time = 0.0;
    m_auton_started = false;
    m_auton_description = "";

    m_publish_count = 0;

    m_robot_phase = INIT;
    if (gsi::Path::pathExists("/media/sda1"))
    {
        gsu::PathUtil::setPath("logs", "/media/sda1/robot/logs");
    }
    else
    {
        gsu::PathUtil::setPath("logs", "/robot/logs");
    }

}

/*******************************************************************************
 *
 ******************************************************************************/
FlightBase::~FlightBase()
{
    try
    {
        Advisory::pinfo("FlightBase::~FlightBase");

        for(ControlThread *control : robot_controls)
        {
            delete control;
        }
        robot_controls.clear();

        if (m_macro_controller != nullptr)
        {
            delete m_macro_controller;
            m_macro_controller = nullptr;
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::~FlightBase");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::RobotInit(void)
{
    try
    {
        LiveWindow::GetInstance()->DisableAllTelemetry();  // with the hope of offloading processor a bit

        std::string log_path = gsu::PathUtil::getPath("logs", "advisories");
        gsi::Path::createPath(log_path);
        Advisory::addObserver(new AdvisoryStdOut());

        Advisory::addObserver(new AdvisoryFile(gsu::SegmentedFile("/robot/logs/advisories", "message", "txt", 10).openNextSegment()));

        Advisory::pinfo("%s: FlightBase::RobotInit()", gsi::Time::getTimeString(gsi::Time::FORMAT_YMDHMSu).c_str());

        m_macro_controller = new MacroController();
        m_macro_controller->start();

        Advisory::pinfo("FlightBase::Loading parameters");
        gsu::Parameter::loadFile("/robot/config/RobotParameters.xml");

        LoadRobot("/robot/config/RobotControl.xml");


        Advisory::pinfo("\n########################### Initializing Robot Controls ###########################");
        for(ControlThread *control : robot_controls)
        {
            control->doControlInit();
        }

        Advisory::pinfo("\n############################ Starting Robot Controls ############################");
        for(ControlThread *control : robot_controls)
        {
            control->start();
        }
        Advisory::pinfo("\n##################################################################################\n");

    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::RobotInit");
    }
}

/******************************************************************************
 * Loads the robot
 ******************************************************************************/
void FlightBase::LoadRobot(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err;
    err = doc.LoadFile(file.c_str());
    if(err != tinyxml2::XML_SUCCESS)
    {
        Advisory::pwarning("could not read macro file %s", file.c_str());
        return;
    }
    tinyxml2::XMLElement *comp;
    const char *name;

    doc.LoadFile(file.c_str());
    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");

    Advisory::pinfo("############################## Loading Robot ##############################");

    if (robot == nullptr)
    {
        Advisory::pwarning("could not read config file %s, no \"robot\" element", file.c_str());
    }
    else
    {
        name = robot->Attribute("name");
        if (name == nullptr)
        {
            Advisory::pinfo("robot: unknown");
        }
        else
        {
            Advisory::pinfo("robot: %s", name);
        }

        Advisory::pinfo("========================= Loading User Interface =========================");
        tinyxml2::XMLElement *oi_xml = robot->FirstChildElement("interface");
        if (oi_xml == NULL)
        {
            Advisory::pwarning("could not initialize OIController, no xml element named \"interface\"");
        }
        else
        {
            OIController::configure(oi_xml);
        }

        comp = robot->FirstChildElement("oi");
        while (comp != NULL)
        {
            name = comp->Attribute("name");
            if (name != NULL)
            {
            }
            comp = comp->NextSiblingElement("oi");
        }

        Advisory::pinfo("========================= Loading Robot Controls =========================");

        tinyxml2::XMLElement *ctrl_xml = robot->FirstChildElement("control");
        {
            while (ctrl_xml != nullptr)
            {
                const char* type = ctrl_xml->Attribute("type");
                const char *name = ctrl_xml->Attribute("name");

                if (name == nullptr)
                {
                    name = "unnamed";
                }

                if (type != nullptr)
                {
                    Advisory::pinfo("\n========================= Creating %s - %s ========================= ", type, name);
                    ControlThread *control = ControlFactory::create(type, ctrl_xml);

                    if (control != nullptr)
                    {
                        robot_controls.push_back(control);
                    }
                    else
                    {
                        Advisory::pinfo("ERROR: failed to create control %s - %s, make sure the \"%s\" type is registered with the factory\n", type, name, type);
                    }

                }
                else
                {
                    Advisory::pinfo("ERROR: could not create control, type is NULL");
                }

                ctrl_xml = ctrl_xml->NextSiblingElement("control");
            }
        }

        Advisory::pinfo("==================== Loading Auton File Names ====================");
        comp = robot->FirstChildElement("auton");
        while (comp != nullptr)
        {
            const char *auton_name = comp->Attribute("name");
            const char *auton_file = comp->Attribute("file");

            if ((auton_name != nullptr) && (auton_file != nullptr))
            {
                if (0 == m_auton_name_list.size())
                {
                    m_auton_chooser.SetDefaultOption(auton_name, new AutonIndex(m_auton_name_list.size())); 
                }
                else
                {
                    m_auton_chooser.AddOption(auton_name, new AutonIndex(m_auton_name_list.size()));
                }

                m_auton_name_list.push_back(auton_name);
                m_auton_file_list.push_back(auton_file);
            }
            else
            {
                Advisory::pwarning("could not parse auton tag");
            }
            comp = comp->NextSiblingElement("auton");
        }

        m_auton_file_count = m_auton_name_list.size();
        Advisory::pinfo("found %d named auton files", m_auton_file_count);

        if (isAutonAvailable())
        {
            m_auton_enabled = true;

            frc::SmartDashboard::PutData("Auton Selected: ", &m_auton_chooser);
            frc::SmartDashboard::PutBoolean("Auton Enabled: ", m_auton_enabled);
            frc::SmartDashboard::PutBoolean("Auton Delay Enabled: ", false);
            frc::SmartDashboard::PutNumber("Auton Delay: ", 0.0);
            frc::SmartDashboard::PutString("Auton Description: ", "none");
        }

        Advisory::pinfo("======================= Loading Macros Names =======================");
        comp = robot->FirstChildElement("macro");
        while (comp != NULL)
        {
            const char *macro_name = comp->Attribute("name");
            const char *macro_file = comp->Attribute("file");

            if ((macro_name != nullptr) && (macro_file != nullptr))
            {
                // Don't load them here, they will be loaded in UpdateConfig()
                // m_macro_controller->loadMacro(macro_name, macro_file, this);

                tinyxml2::XMLElement *oi_xml = comp->FirstChildElement("oi");
                while (oi_xml != nullptr)
                {
                    name = oi_xml->Attribute("name");
                    if (name != NULL)
                    {
                        if (strcmp(name, "start") == 0)
                        {
                            Advisory::pinfo("  connecting start macro %s to oi button", macro_name);
                            OIController::subscribeDigital(oi_xml, this, CMD_START_MACRO + m_macro_name_list.size());
                        }
                        else if (strcmp(name, "abort") == 0)
                        {
                            Advisory::pinfo("  connecting abort macro %s to oi button", macro_name);
                            OIController::subscribeDigital(oi_xml, this, CMD_ABORT_MACRO + m_macro_name_list.size());
                        }
                    }
                    oi_xml = oi_xml->NextSiblingElement("oi");
                }

                m_macro_name_list.push_back(macro_name);
                m_macro_file_list.push_back(macro_file);
            }
            else
            {
                Advisory::pwarning("could not load macro file");
            }
            comp = comp->NextSiblingElement("macro");
        }

        Advisory::pinfo("========================== Loading Macros ==========================");
        int macro_index = 0;
        for (std::string file : m_macro_file_list)
        {
            m_macro_controller->loadMacro(m_macro_name_list[macro_index], file /*, this*/);
            macro_index++;
        }

        UpdateAutonConfig();
        // connect subsystems
    }

    Advisory::pinfo("############################## Robot Loaded ##############################");
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::UpdateAutonConfig(void)
{
    try
    {
        if (isAutonAvailable())
        {
            Advisory::pinfo("FlightBase::UpdateAutonConfig()");

            AutonIndex * auton_index = m_auton_chooser.GetSelected();
            if (auton_index != nullptr)
            {
                m_auton_select = auton_index->value();
            }

            m_auton_enabled = frc::SmartDashboard::GetBoolean("Auton Enabled: ", m_auton_enabled);
            m_auton_delay_enabled = frc::SmartDashboard::GetBoolean("Auton Delay Enabled: ", m_auton_delay_enabled);
            m_auton_delay = frc::SmartDashboard::GetNumber("Auton Delay: ", m_auton_delay);

            Advisory::pinfo("FlightBase::UpdateAutonConfig -- enabled=%s, select=%d, delay_enabled=%s, delay=%5.3f",
                    m_auton_enabled?"true":"false", m_auton_select, m_auton_delay_enabled?"true":"false", m_auton_delay);

            if ((m_auton_select >= 0) && (m_auton_select < m_auton_file_count))
            {
                m_macro_controller->loadMacro("auton", m_auton_file_list[m_auton_select] /*, this*/);
                m_auton_select_loaded = m_auton_select;
                m_auton_select_mismatch = 0;
                m_auton_description = m_macro_controller->getMacro("auton")->getDescription();
            }
            else
            {
                m_auton_select = -1;
                m_auton_select_loaded = -1;
                m_auton_enabled = false;
                m_auton_description = "none";
                m_auton_delay_enabled = false;
                m_auton_delay = 0.0;

                frc::SmartDashboard::PutBoolean("Auton Enabled: ", m_auton_enabled);
                frc::SmartDashboard::PutBoolean("Auton Delay Enabled: ", m_auton_delay_enabled);
                frc::SmartDashboard::PutNumber("Auton Delay: ", m_auton_delay);
            }

            //@TODO set lights to indicate which auton selected

            frc::SmartDashboard::PutString("Auton Description: ", m_auton_description);
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::UpdateAutonConfig");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::DisabledInit(void)
{
    m_robot_phase = DISABLED;
    Advisory::pinfo("%s: FlightBase::DisabledInit()", gsi::Time::getTimeString(gsi::Time::FORMAT_YMDHMSu).c_str());

    try
    {
        m_macro_controller->abortAll();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::DisabledInit while aborting macros");
    }

    for(ControlThread *control : robot_controls)
    {
        try
        {
            control->doDisabledInit();
        }
        catch (...)
        {
            Advisory::pwarning("exception caught in FlightBase::DisabledInit while calling controls");
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::DisabledPeriodic(void)
{
    try
    {
        OIController::update();

        if (isAutonAvailable())
        {
            AutonIndex * auton_index = m_auton_chooser.GetSelected();
            if (auton_index != nullptr)
            {
                m_auton_select = auton_index->value();
            }

            if ((m_auton_select != m_auton_select_loaded) && (m_auton_select >= 0)
                    && (m_auton_select < m_auton_file_count))
            {
                // 40 cycles should be about 2 seconds, that should be enough
                // time for the user to finish adjusting the auton selection
                if(m_auton_select_mismatch++ > 40)
                {
                    UpdateAutonConfig();
                }
            }
        }
        publish();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::DisabledPeriodic");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::AutonomousInit(void)
{
    m_robot_phase = AUTON;
    Advisory::pinfo("%s: FlightBase::AutonomousInit()", gsi::Time::getTimeString(gsi::Time::FORMAT_YMDHMSu).c_str());

    try
    {
        m_macro_controller->abortAll();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::AutonomousInit while aborting macros");
    }

    try
    {
        m_auton_enabled = frc::SmartDashboard::GetBoolean("Auton Enabled: ", m_auton_enabled);
        m_auton_delay_enabled = frc::SmartDashboard::GetBoolean("Auton Delay Enabled: ", m_auton_delay_enabled);
        m_auton_delay = frc::SmartDashboard::GetNumber("Auton Delay: ", m_auton_delay);

        m_auton_started = false;
        if (m_auton_enabled)
        {
            m_auton_start_time = GetTime();

            if (m_auton_delay_enabled)
            {
                m_auton_start_time += m_auton_delay;
            }
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::AutonomousInit while initializing auton");
    }

    for(ControlThread *control : robot_controls)
    {
        try
        {
            control->doAutonomousInit();
        }
        catch (...)
        {
            Advisory::pwarning("exception caught in FlightBase::AutonomousInit while calling controls");
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::AutonomousPeriodic(void)
{
    try
    {
        OIController::update();

        if ((m_auton_enabled) && (GetTime() >= m_auton_start_time) && (! m_auton_started))
        {
            m_macro_controller->startMacro("auton");
            m_auton_started = true;
        }

        publish();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::AutonomousPeriodic");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::TeleopInit(void)
{
    m_robot_phase = TELEOP;
    Advisory::pinfo("%s: FlightBase::TeleopInit()", gsi::Time::getTimeString(gsi::Time::FORMAT_YMDHMSu).c_str());

    try
    {
        m_macro_controller->abortAll();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::TeleopInit while aborting macros");
    }

    for(ControlThread *control : robot_controls)
    {
        try
        {
            control->doTeleopInit();
        }
        catch (...)
        {
            Advisory::pwarning("exception caught in FlightBase::TeleopInit while calling controls");
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::TeleopPeriodic(void)
{
    try
    {
        OIController::update();
        publish();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::TeleopPeriodic");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::TestInit(void)
{
    m_robot_phase = TEST;
    Advisory::pinfo("%s: FlightBase::TestInit()", gsi::Time::getTimeString(gsi::Time::FORMAT_YMDHMSu).c_str());

    try
    {
        m_macro_controller->abortAll();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::TestInit while aborting macros");
    }

    for(ControlThread *control : robot_controls)
    {
        try
        {
            control->doTestInit();
        }
        catch (...)
        {
            Advisory::pwarning("exception caught in FlightBase::TestInit while calling controls");
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::TestPeriodic(void)
{
    try
    {
        OIController::update();
        publish();
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::TestPeriodic");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::setAnalog(int id, float val)
{
    try
    {
        //        analog values can change very often so only use this Advisory
        //        when doing specific debugging
        //        Advisory::pinfo("Robot::setAnalog(%d, %f)", id, val);

        switch (id)
        {
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::setAnalog");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::setDigital(int id, bool val)
{
    try
    {
        Advisory::pinfo("FlightBase::setDigital(%d, %d)", id, val);

        switch(id)
        {
            default:
                {
                    if ((id >= CMD_START_MACRO) && (id < CMD_START_MACRO_MAX) &&
                            (m_robot_phase != INIT) && (m_robot_phase != DISABLED))
                    {
                        if (val)
                        {
                            m_macro_controller->startMacro(m_macro_name_list[id - CMD_START_MACRO]);
                        }
                    }
                    else if ((id >= CMD_ABORT_MACRO) && (id < CMD_ABORT_MACRO_MAX) &&
                            (m_robot_phase != INIT) && (m_robot_phase != DISABLED))
                    {
                        if (val)
                        {
                            m_macro_controller->abortMacro(m_macro_name_list[id - CMD_ABORT_MACRO]);
                        }
                    }

                } break;
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::setDigital");
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::setInt(int id, int val)
{
    try
    {
        Advisory::pinfo("FlightBase::setInt(%d, %d)", id, val);
        switch (id)
        {
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::setInt");
    }
}

/******************************************************************************
 *
 * return true if at least one auton file was loaded
 *
 ******************************************************************************/
inline bool FlightBase::isAutonAvailable()
{
    return m_auton_file_count > 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
void FlightBase::publish(void)
{
    try
    {
        if (m_publish_count++ >= 30)
        {

            //
            // Update Dashboard Values
            //
#ifdef DEBUG_PUBLISH
            frc::SmartDashboard::PutNumber("Macro Starts: ", this->m_macro_controller->getMacroStarts());
            frc::SmartDashboard::PutNumber("Macro Completes: ", this->m_macro_controller->getMacroCompletes());
            frc::SmartDashboard::PutNumber("Macro Aborts: ", this->m_macro_controller->getMacroAborts());
            frc::SmartDashboard::PutNumber("Macros Running: ", this->m_macro_controller->getMacrosRunning());
            //@TODO Update Lights lights to indicate how many macros running
#endif
            m_publish_count = 0;
        }

        if (m_publish_count < robot_controls.size())
        {
            robot_controls[m_publish_count]->doPublish();
        }
    }
    catch (...)
    {
        Advisory::pwarning("exception caught in FlightBase::publish");
    }
}

