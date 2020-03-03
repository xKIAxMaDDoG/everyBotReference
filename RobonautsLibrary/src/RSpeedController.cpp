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
#include "RobonautsLibrary/RSpeedController.h"

/******************************************************************************
 * 
 * Create an interface of RSpeedController, geto an APS plugged into the specified channel.
 * 
 * @param	channel	the channel or port that the pot is in
 * 
 ******************************************************************************/
RSpeedController::RSpeedController(int port_device)
{
    m_device_id = port_device;
    m_mode = ControlModeType::DUTY_CYCLE;
    m_sensor_invert = 1.0;  // 1.0 if not inverted, -1.0 if inverted
    m_output_per_count = 1.0;
    m_output_offset = 0.0;
}

RSpeedController::~RSpeedController(void) {} 
RSpeedController::ControlModeType RSpeedController::GetControlMode() { return m_mode; }

void RSpeedController::SensorOutputPerCount(double conversion, double offset)
{
    m_output_per_count = conversion;
    m_output_offset = offset;
}

bool RSpeedController::isForwardLimitPressed(void) { return false; }
bool RSpeedController::isReverseLimitPressed(void) { return false; }
