/*******************************************************************************
 *
 * File: RobotMain.cpp
 *
 * Written by:   
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#include "RobotMain.h"

/******************************************************************************
 *
 *
 *
 ******************************************************************************/
RobotMain::RobotMain(void) : PacbotBase()
{
    RegisterControls();
}

/******************************************************************************
 *
 *
 *
 ******************************************************************************/
RobotMain::~RobotMain()
{
}

//
// This pre-processor macro is what makes this class the class that is run
// as a robot main.
//
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<RobotMain>(); }
#endif
