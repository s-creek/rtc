// -*- C++ -*-
/*!
 * @file  creekPhysicalCamera.cpp * @brief creekPhysicalCamera * $Date$ 
 *
 * $Id$ 
 */
#include "creekPhysicalCamera.h"

// Module specification
static const char* creekphysicalcamera_spec[] =
  {
    "implementation_id", "creekPhysicalCamera",
    "type_name",         "creekPhysicalCamera",
    "description",       "creekPhysicalCamera",
    "version",           "1.0",
    "vendor",            "skj",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekPhysicalCamera",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekPhysicalCamera::creekPhysicalCamera(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_imageOut("image", m_image),
    m_creekPhysicalCameraServicePort("creekPhysicalCameraService")
{
  m_service0.setComponent(this);
  m_show = true;
}


creekPhysicalCamera::~creekPhysicalCamera()
{
}


RTC::ReturnCode_t creekPhysicalCamera::onInitialize()
{
  // Set OutPort buffer
  addOutPort("image", m_imageOut);

  // Set service provider to Ports
  m_creekPhysicalCameraServicePort.registerProvider("service0", "creekPhysicalCameraService", m_service0);
  addPort(m_creekPhysicalCameraServicePort);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPhysicalCamera::onActivated(RTC::UniqueId ec_id)
{
  // init camera
  if( !m_camera.open(0) ) {
    std::cerr << "PhysicalCamera : error, could'nt open camera" << std::endl;
    return RTC::RTC_ERROR;
  }


  // init image
  m_camera >> m_frame;
  if( m_frame.empty() ) {
    std::cerr << "PhysicalCamera : error, could'nt get image" << std::endl;
    return RTC::RTC_ERROR;
  }
  m_image.data.image.width  = m_frame.cols;
  m_image.data.image.height = m_frame.rows;
  m_image.data.image.raw_data.length(m_frame.total() * m_frame.channels());
  switch(m_frame.channels())
    {
    case 1:
      m_image.data.image.format = Img::CF_GRAY;
      break;
    case 3:
      m_image.data.image.format = Img::CF_RGB;
      break;
    default:
      m_image.data.image.format = Img::CF_UNKNOWN;
    }


  std::cout << "PhysicalCamera : onActivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPhysicalCamera::onDeactivated(RTC::UniqueId ec_id)
{
  cv::destroyWindow( "image" );
  m_camera.release();

  std::cout << "PhysicalCamera : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPhysicalCamera::onExecute(RTC::UniqueId ec_id)
{
  if( m_camera.grab() ) {
    m_camera.retrieve(m_frame);

    if( m_frame.channels() == 3 )
      cv::cvtColor(m_frame, m_frame, CV_BGR2RGB);
    
    
    memcpy(m_image.data.image.raw_data.get_buffer(),
	   m_frame.data,
	   m_image.data.image.raw_data.length());
    
    m_imageOut.write();

    if( m_show )
      cv::imshow( "image", m_frame );
    else
      cv::destroyWindow( "image" );

    cv::waitKey(1);
  }
  return RTC::RTC_OK;
}


void creekPhysicalCamera::show()
{
  m_show != m_show;
}


extern "C"
{
 
  void creekPhysicalCameraInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekphysicalcamera_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekPhysicalCamera>,
                             RTC::Delete<creekPhysicalCamera>);
  }
  
};



