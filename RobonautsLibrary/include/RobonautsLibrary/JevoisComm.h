#pragma once
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <vector>
#include "frc/SerialPort.h"

/**
 * @brief Class JevoisComm
 * Sends information over the serial port from the Jevois Camera
 * Contains essential Jevois communication
 */
class JevoisComm
{
public:

    typedef struct
    {
        double x;
        double y;
        double theta;
    } targetLocation_t;
  /**
 * @brief Construct a new Jevois Comm object
 *Sets all the default settings for the Jevois Camera
 * 
 * @param portA 
 *  portA uses USB port 1, closest to the top of the RoboRio
 */
  JevoisComm(int32_t portA);
  /**
 * @brief Construct a new Jevois Comm object
 * Another version of the default settings
 */
  JevoisComm();
  /**
 * @brief Destroy the Jevois Comm object
 * Does nothing
 */
  ~JevoisComm();
  /**
 * @brief Get the Target Locs object
 * Returns a list of target locations
 * 
 * @param locations 
 * @return true 
 * @return false 
 * returns true if there is a target
 * returns false if there isnt
 */
  bool getTargetLocs(std::vector<JevoisComm::targetLocation_t> &locations);
  /**
 * @brief Void Init
 * Can change usb port being used on RoboRio
 * @param portA 
 * If portA is 1, USB port 1 is used
 * Otherwise USB port 2 is used
 */
  void init(int32_t portA);
  void calibrate(uint32_t num_samplesA);

/**
 * @brief uint32 getPacketCount
 * returns the number of packets received
 */
    uint32_t getPacketCount();

private:
  /**
 * @brief Function shared_ptr
 *Activates only when the camera sees a target
 */
    
    frc::SerialPort* m_serial_ptr;
    uint32_t m_num_bad;
    uint32_t m_max_num_bad;
    uint32_t m_packet_count;

    

/**
 * @brief Function update
 * Runs through the process of getting the locations using m_camera_ptr
 * Takes and stores the locations from the serial port
 */
  void update();
  /**
 * @brief Function loop
 * Open loop that sets the format and turns on the camera
 * Uses update, and sleeps for 15 milliseconds for every cycle
 */
  void loop();
  /**
 * @brief 
 * Temporarily stores data from the serial port until it can be parsed
 */
  char m_buffer[200];
  /**
 * @brief 
 * A condition to let the locations be returned in getTargetLocs
 */
  bool m_valid;
  /**
 * @brief 
 * Correct locations
 */
  int32_t m_valid_locations_idx;
  /**
 * @brief 
 * Stores the number of locations
 * 
 */
  int32_t m_new_locations_idx;
  /**
 * @brief 
 * Tells which port to use 
 * 
 */
  frc::SerialPort::Port m_port;
  /**
 * @brief 
 * 
 * 
 */
  float m_max_fit;
  /**
 * @brief 
 * Camera id
 */
  int32_t m_camera_id;
  /**
 * @brief 
 * 
 */
  int32_t m_port_num;
  /**
 * @brief 
 * Width of Jevois output in pixels
 * 
 */
  int32_t m_width;
  /**
 * @brief 
 * Height of Jevois out in pixels
 * 
 */
  int32_t m_height;
  /**
 * @brief 
 * Frames per second of Jevois output
 * 
 */

  int32_t m_fps;

  uint32_t m_num_samples;
  uint32_t m_num_samples_taken;
  bool m_calibration_active;
  float m_cal_values[3];
  float m_camera_rotation_sin;
  float m_camera_rotation_cos;
  

  /**
 * @brief 
 * Thread that detaches from Port 2 if port 2 is plugged in
 * 
 */
  std::shared_ptr<std::thread> m_thread_ptr;
  /**
 * @brief 
 * Initializes loc
 */
  std::vector<JevoisComm::targetLocation_t> m_locations[2];
  /**
 * @brief Method readLine
 * parses the buffer data
 * 
 * @param bufferA 
 * data from serial
 * @param buffer_sizeA 
 * size of data
 * @return int32_t 
 */
  int32_t readLine(char bufferA[], int32_t buffer_sizeA);
  /**
 * @brief Method cameraIdFromDev
 * get the camera name and id
 * 
 * @param devA 
 * @param camera_idA 
 * @return int32_t 
 */
  int32_t cameraIdFromDev(std::string &devA, int32_t &camera_idA);
};
