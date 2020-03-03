/*******************************************************************************
 *
 * File: CameraServerControl.cpp
 * 
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#include "RobonautsLibrary/RobotUtil.h"
#include "RobonautsLibrary/XmlRobotUtil.h"
#include "gsu/Advisory.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "RobonautsControls/SimpleCameraServer.h"

using namespace tinyxml2;
using namespace frc;

/*******************************************************************************	
 * 
 * Create an instance of a motor control and connect it to the specified
 * motor and inputs
 * 
 ******************************************************************************/
SimpleCameraServer::SimpleCameraServer(std::string control_name, XMLElement* xml)
	: ControlThread(control_name)
{
	XMLElement *comp;
	Advisory::pinfo("========================= Creating Camera Server Control [%s] =========================",
	            control_name.c_str());

	m_selected_camera = 0;

	const char *name = nullptr;

	//
	// Parse XML
	//
	comp = xml-> FirstChildElement("camera");
	while (comp != nullptr)
	{
        Advisory::pinfo("  initializing camera");

	    m_camera_id = -1;

	    comp->QueryIntAttribute(("id"), &m_camera_id);
	    if ((m_camera_id >= 0) && (m_camera_id < 2))
	    {
            m_camera_res_width = 160;
            m_camera_res_height = 120;
            m_camera_fps = 10;
			std::string camera_name = "Camera";

            comp->QueryIntAttribute("res_width", &m_camera_res_width);
            comp->QueryIntAttribute("res_height", &m_camera_res_height);
            comp->QueryIntAttribute("fps", &m_camera_fps);

			std::string camera_device_name;

   				if(m_camera_id == 0)
   				 {
      				m_port = frc::SerialPort::Port::kUSB1;
                    camera_device_name = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0";
    			 }
                else if(m_camera_id == 1)
                 {
                    m_port = frc::SerialPort::Port::kUSB2;
                    camera_device_name = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0";
                 }
				else
				{
					Advisory::pwarning("CameraServerControl too many cameras \"id\" attribute of %d", m_camera_id);
				}
            cameraIdFromDev(camera_device_name, m_camera_id);
 
            
	    }
	    else
	    {
	        Advisory::pwarning("CameraServerControl ignoring camera specification with invalid \"id\" attribute of %d", m_camera_id);
	    }

		comp = comp->NextSiblingElement("camera");
	}

	comp = xml-> FirstChildElement("oi");
	while (comp != nullptr)
	{
		name = comp->Attribute("name");
		if (name != nullptr)
		{
			if (strcmp(name, "toggle") == 0)
			{
				Advisory::pinfo("  connecting toggle channel");
				OIController::subscribeDigital(comp, this, CMD_TOGGLE);
			}
			else if (strcmp(name, "select") == 0)
			{
				Advisory::pinfo("  connecting select channel");
				OIController::subscribeDigital(comp, this, CMD_SELECT);
			}
			else
			{
                Advisory::pinfo("  unknown oi ignored name=\"%s\"", name);
			}
		}
		
		comp = comp->NextSiblingElement("oi");
	}

//    m_video_server = CameraServer::GetInstance()->GetServer();
}

/*******************************************************************************	
 * 
 * Release any resources allocated by this object
 * 
 ******************************************************************************/
SimpleCameraServer::~SimpleCameraServer(void)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void SimpleCameraServer::controlInit(void)
{
}

/*******************************************************************************
 *
 * This is the callback for OIController::subscribeAnalog, if the XML config 
 * specifies an analog input, the constructor of this object will connect that 
 * input to this method.
 * 
 * @param id	the control id passed to the subscribe
 * @param val	the new value of the analog channel that was subscribed to
 * 
 ******************************************************************************/
void SimpleCameraServer::setAnalog(int id, float val)
{
}

/*******************************************************************************	
 *
 * This is the callback for OIController::setDigital, if the XML config 
 * specifies an increment, decrement, or stop input, the constructor 
 * of this object will connect that/those input(s) to this method.
 * 
 * @param id	the control id passed to the subscribe
 * @param val	the new value of the digital channel that was subscribed to
 * 
 ******************************************************************************/
void SimpleCameraServer::setDigital(int id, bool val)
{
//    Advisory::pinfo("CameraServerControl::setDigital(%d, %s)", id, (val?"true":"false"));
/*	int previous_selected = m_selected_camera;

	switch (id)
	{
		case CMD_TOGGLE:
		{
			if (val)
			{
			    if (m_selected_camera == 0)
			    {
			        m_selected_camera = 1;
			    }
			    else
			    {
			        m_selected_camera = 0;
			    }
			    if (m_selected_camera != previous_selected)
			    {
			    	Advisory::pinfo("selecting camera %d, %s, %s, %s, %s", m_selected_camera,
			        m_camera[m_selected_camera].GetName().c_str(), m_camera[m_selected_camera].GetPath().c_str(),
			        m_camera[m_selected_camera].IsConnected()?"true":"false", m_camera[m_selected_camera].GetDescription().c_str());
			    }

			    m_video_server.SetSource(m_camera[m_selected_camera]);
				*/
//	            NetworkTable::GetTable("")->PutString("CameraSelection", m_camera[m_selected_camera].GetName());
//			    CS_Status set_status = m_video_server.GetLastStatus();
//			    cs::VideoSource check_source = m_video_server.GetSource();
//			    Advisory::pinfo("checking camera x, %s, %s, %s  status=%d",
//			        check_source.GetName().c_str(),
//			        check_source.IsConnected()?"true":"false", check_source.GetDescription().c_str(), set_status);
/*			}
		} break;
			
		case CMD_SELECT:
		{
			if (val)
			{
	            m_selected_camera = 1;
			}
			else
			{
			    m_selected_camera = 0;
			}
			if (m_selected_camera != previous_selected)
			{
				Advisory::pinfo("selecting camera %d, %s, %s, %s, %s", m_selected_camera,
		        m_camera[m_selected_camera].GetName().c_str(), m_camera[m_selected_camera].GetPath().c_str(),
		        m_camera[m_selected_camera].IsConnected()?"true":"false", m_camera[m_selected_camera].GetDescription().c_str());
			}

		    m_video_server.SetSource(m_camera[m_selected_camera]);
//		    NetworkTable::GetTable("")->PutString("CameraSelection", m_camera[m_selected_camera].GetName());
//            CS_Status set_status = m_video_server.GetLastStatus();
//            cs::VideoSource check_source = m_video_server.GetSource();
//            Advisory::pinfo("checking camera x, %s, %s, %s  status=%d",
//                check_source.GetName().c_str(),
//                check_source.IsConnected()?"true":"false", check_source.GetDescription().c_str(), set_status);
		} break;

		default:
			break;
	}*/

}

/*******************************************************************************
 *
 ******************************************************************************/
void SimpleCameraServer::setInt(int id, int val)
{
}

/*******************************************************************************	
 *
 ******************************************************************************/
void SimpleCameraServer::publish()
{
    SmartDashboard::PutNumber(getName() +" selected camera: ", m_selected_camera);
}

/*******************************************************************************
 *
 * This method will be called once a period to do anything that is needed,
 * in this case it just sets the motor power to the current value.
 * 
 ******************************************************************************/
void SimpleCameraServer::run()
{

	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(m_camera_id);

	// try to get the desired resolution and FPS.  If that doesn't work, will re-size as needed
	camera.SetResolution(m_camera_res_width, m_camera_res_height);
	//camera.SetPixelFormat(cs::VideoMode::kYUYV);
	camera.SetFPS(m_camera_fps);

	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(camera);
	cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Driver Camera", m_camera_res_width, m_camera_res_height);
	cv::Mat source;
	cv::Mat output;
	double init_time, delta_time;
    while(true) 
	{
		init_time = getPhaseElapsedTime();
        cvSink.GrabFrame(source);
//		Advisory::pinfo("delta time end %p %.8f", &source, getPhaseElapsedTime()-init_time);
		try
		{
			if(source.cols != m_camera_res_width || source.rows != m_camera_res_height)
			{
				cv::resize(source, output, cv::Size(m_camera_res_width, m_camera_res_height));
    	        outputStreamStd.PutFrame(output);
//				Advisory::pinfo("resized [%d, %d], [%d, %d]", source.cols, m_camera_res_width, source.rows, m_camera_res_height);
			}
			else
			{
    	        outputStreamStd.PutFrame(source);
//				Advisory::pinfo("straight through");
			}
		}
		catch (...)
		{
			Advisory::pinfo("SimpleCameraServer:: failed to convert and put frame");
		}
		delta_time = getPhaseElapsedTime() - init_time;
		int ms;
		ms =1000.0/m_camera_fps - 1000 * delta_time;  // 1000/fps is nominal slepp + 1000 * delta_time reduces it by time to pull and process frame.
		if(ms < 0)
		{
			ms = 10;
		}
//		Advisory::pinfo("delta time end %d, %.8f", ms, getPhaseElapsedTime()-init_time);
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}
	
}
int32_t SimpleCameraServer::cameraIdFromDev(std::string &devA,int32_t &m_camera_idA)
{
    int32_t err = -1;
    int32_t id = -1;
    char *ret = NULL;
    char path[1000];
    ret = realpath(devA.c_str(),path);
    if (ret != NULL)
    {
        fprintf(stderr, "%s\n",path);
        sscanf(path,"/dev/video%d", &id);
        m_camera_idA = id;
        err = 0;
    }
    return(err);
}
