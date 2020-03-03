#include "cscore_oo.h"
#include "cameraserver/CameraServer.h"
#include "RobonautsLibrary/JevoisComm.h"
#include <gsu/Advisory.h>
#include <gsu/Parameter.h>

JevoisComm::JevoisComm(int32_t portA)
    : m_serial_ptr(nullptr), m_valid(false), m_valid_locations_idx(-1), m_new_locations_idx(0), m_width(160), m_height(120), m_fps(30.0), m_thread_ptr(nullptr)
{
    init(portA);
}

JevoisComm::JevoisComm()
    : m_serial_ptr(nullptr), m_valid(false), m_valid_locations_idx(-1), m_new_locations_idx(0), m_thread_ptr(nullptr)
{
}

void JevoisComm::init(int32_t portA)
{
    m_packet_count = 0;
    m_port_num = portA;
    m_num_bad = 0;
    m_max_num_bad = 6;
    m_serial_ptr = nullptr;
    m_calibration_active = false;
    m_thread_ptr = std::make_shared<std::thread>(&JevoisComm::loop, this);
    m_thread_ptr->detach();
    m_cal_values[0] = 0.0;
    m_cal_values[1] = 0.0;
    m_cal_values[2] = 0.0;
    float camera_rotation = gsu::Parameter::get("TARGET_CAM_THETA_OFFSET", 0.0F);
    m_camera_rotation_sin = sin(camera_rotation);
    m_camera_rotation_cos = cos(camera_rotation);
}

JevoisComm::~JevoisComm()
{
}

int32_t JevoisComm::cameraIdFromDev(std::string &devA, int32_t &camera_idA)
{
    int32_t err = -1;
    int32_t id = -1;
    char *ret = NULL;
    char path[1000];
    ret = realpath(devA.c_str(), path);
    if (ret != NULL)
    {
        sscanf(path, "/dev/video%d", &id);
        camera_idA = id;
        err = 0;
    }
    return (err);
}

void JevoisComm::loop()
{
    std::string camera_name;

    if (m_port_num == 1)
    {
        m_port = frc::SerialPort::Port::kUSB1;
        camera_name = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0";
    }
    else
    {
        m_port = frc::SerialPort::Port::kUSB2;
        camera_name = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0";
    }

    cameraIdFromDev(camera_name, m_camera_id);
    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture("Jevois", m_camera_id);
    camera.SetPixelFormat(cs::VideoMode::PixelFormat::kYUYV);
    camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

    while (true)
    {
        update();
        if (m_num_bad >= m_max_num_bad)
        {
            delete m_serial_ptr;
            m_serial_ptr = nullptr;
            m_locations[m_new_locations_idx].clear();
            m_valid = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
}

void JevoisComm::update()
{
    int32_t num_bytes_read = 0;
    char key = 0x00;
    char buffer[100];
    char name[20];
    int32_t x, y, theta, idx, num, fit;
    targetLocation_t loc;

    if (m_serial_ptr == nullptr)
    {
        m_serial_ptr = new frc::SerialPort(115200, m_port);
        m_serial_ptr->EnableTermination('n');
    }
    if (m_serial_ptr != nullptr)
    {
        num_bytes_read = m_serial_ptr->Read(&key, 1);
        if (num_bytes_read <= 0)
        {
            ++m_num_bad;
        }
        else
        {
            m_num_bad = 0;
            m_packet_count++;
            switch (key)
            {
            case 'N':
            {
                char dim;
                num_bytes_read = m_serial_ptr->Read(&dim, 1);
                switch (dim)
                {
                case '3':
                    memset(buffer, 0, sizeof(buffer));
                    num_bytes_read = readLine(buffer, sizeof(buffer));
                    if (num_bytes_read > 0)
                    {
                        sscanf(buffer, " %s %d %d %d %d %d %d", name, &x, &y, &theta, &idx, &num, &fit);
                        //fprintf(stderr, " %s\n", buffer);
                        //fprintf(stderr, " %s %d %d %d %d %d %d\n", name, x, y, theta, idx, num, fit);
                       // if (fit < m_max_fit)
                        {
                            loc.x = (double)x / 100.0;
                            loc.y = (double)y / 100.0;
                            loc.theta = (double)theta / 100.0;
                            if (m_calibration_active)
                            {
                                m_cal_values[0] = (m_cal_values[0] * m_num_samples_taken + loc.x) / (m_num_samples_taken + 1);
                                m_cal_values[1] = (m_cal_values[1] * m_num_samples_taken + loc.y) / (m_num_samples_taken + 1);
                                m_cal_values[2] = (m_cal_values[2] * m_num_samples_taken + loc.theta) / (m_num_samples_taken + 1);
                                ++m_num_samples_taken;
                                //printf("Samples: %d \n", m_num_samples_taken);
                                if (m_num_samples_taken >= m_num_samples)
                                {  
                                    float camera_rotation = -atan2(m_cal_values[1],m_cal_values[0]);
                                    m_calibration_active = false;
                                    m_camera_rotation_sin = sin(camera_rotation);
                                    m_camera_rotation_cos = cos(camera_rotation);
                                    gsu::Parameter::put("TARGET_CAM_THETA_OFFSET", static_cast<float>(camera_rotation));

                                    Advisory::pinfo("***********************************************************************\n\n");
                                   Advisory::pfatal("Calibration : %f, %f, %f", camera_rotation, m_camera_rotation_sin, m_camera_rotation_cos);
                                   
                                    Advisory::pinfo("/n***********************************************************************\n");
                                    m_calibration_active = false;
                                    m_camera_rotation_sin = sin(camera_rotation);
                                    m_camera_rotation_cos = cos(camera_rotation);
                                }
                                else
                                {
                                    double xx = loc.x;
                                    loc.x = xx*m_camera_rotation_cos - loc.y*m_camera_rotation_sin;
                                    loc.y = xx*m_camera_rotation_sin + loc.y*m_camera_rotation_cos;
                                }
                                
                            }
                            if (idx == 0) m_locations[m_new_locations_idx].clear();
                            
                            double xx = loc.x;
                            loc.x = xx*m_camera_rotation_cos - loc.y*m_camera_rotation_sin;
                            loc.y = xx*m_camera_rotation_sin + loc.y*m_camera_rotation_cos;
                            m_locations[m_new_locations_idx].push_back(loc);
                        }
                        if (idx >= (num - 1))
                        {
                            //                            fprintf(stderr, "all locations full new _locations - %d", m_new_locations_idx);
                            m_valid_locations_idx = m_new_locations_idx;
                            m_new_locations_idx = !m_new_locations_idx;
                            m_locations[m_new_locations_idx].clear();

                            m_valid = true;
                        }
                    }
                    break;
                case '1':
                    num_bytes_read = readLine(buffer, sizeof(buffer));
                    //                    fprintf(stderr, "No Target\n");
                    if (num_bytes_read > 0)
                    {
                        m_valid = false;
                    }
                    break;
                }
            }
            case 'T':
                break;
            }
        }
    }
}

int32_t JevoisComm::readLine(char bufferA[], int32_t buffer_sizeA)
{
    char c = 0x00;
    int32_t idx = 0;
    bool done = false;
    int32_t n_bytes = 0;
    while (!done)
    {
        n_bytes = m_serial_ptr->Read(&c, 1);
        if (n_bytes > 0)
        {
            if ((n_bytes < 1) || (c == 0x0A) || (c == 0x0D) || (c == 0x00))
            {
                done = true;
            }
            else
            {
                bufferA[idx] = c;
                ++idx;
                if (idx >= buffer_sizeA)
                {
                    done = true;
                    idx = -1;
                }
            }
        }
    }
    return (idx);
}

bool JevoisComm::getTargetLocs(std::vector<JevoisComm::targetLocation_t> &locations)
{
    if (m_valid && (m_valid_locations_idx >= 0))
    {
        locations = m_locations[m_valid_locations_idx];
        //        fprintf(stderr, "Num locations %d\n", (int32_t)locations.size());
    }
    m_valid = false;
    return (m_locations);
}

void JevoisComm::calibrate(uint32_t num_samplesA)
{
    m_num_samples = num_samplesA;
    m_num_samples_taken = 0;
    m_calibration_active = true;
    memset(m_cal_values, 0, sizeof(m_cal_values));
}

uint32_t JevoisComm::getPacketCount()
{
    return(m_packet_count);
}
