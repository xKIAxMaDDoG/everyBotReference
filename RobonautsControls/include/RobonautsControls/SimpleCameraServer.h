/*******************************************************************************
 *
 * File: CameraServerControl.h
 * 
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "RobonautsControls/PeriodicControl.h"
#include "RobonautsLibrary/OIObserver.h"
#include "RobonautsLibrary/OIController.h"
#include "frc/SerialPort.h"

#include "cscore_oo.h"
#include "cameraserver/CameraServer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

/*******************************************************************************	
 * 
 * Create an instance of camera server control and connect it to the specified
 * user inputs
 * 
 * This class is designed to be created from an XML element with the following
 * format, portions contained in [ ] are optional.
 *  
 *  <control type="camera_server" >
 *      [<camera id="0" [res_x="160"] [res_y="120"] [fps="15"] />]
 * 		[<camera id="1" [res_x="160"] [res_y="120"] [fps="15"] />]
 *
 *      [<oi name="toggle"	device="pilot" chan="1" [invert="false"] />]
 *      [<oi name="select"	device="pilot" chan="5" [invert="false"] />]
 *  </control>
 *
 ******************************************************************************/
class SimpleCameraServer : public ControlThread, public OIObserver
{
	public:
		enum {CMD_TOGGLE=0, CMD_SELECT};
		
		SimpleCameraServer(std::string control_name, tinyxml2::XMLElement *xml);
		~SimpleCameraServer(void);

  		void controlInit(void);

		void setAnalog(int id, float val);
		void setDigital(int id, bool val);
		void setInt(int id, int val);

		void publish(void);

		void run(void);


	private:
		cs::UsbCamera m_camera;
		cs::CvSink m_video_sink;
		cs::CvSource m_outputStream;
		cv::Mat m_source_frame;
		cv::Mat m_output_frame;
		uint8_t m_selected_camera;
		int32_t cameraIdFromDev(std::string &devA,int32_t &camera_idA);
		frc::SerialPort::Port m_port;

		int32_t m_camera_res_width;
        int32_t m_camera_res_height;
    	int32_t m_camera_fps;
		int32_t m_camera_id;

		const int32_t c_image_time_ms = 35;

};
