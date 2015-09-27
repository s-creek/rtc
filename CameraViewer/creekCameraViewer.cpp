// -*- C++ -*-

#include "creekCameraViewer.h"

// Module specification
static const char* creekcameraviewer_spec[] =
  {
    "implementation_id", "creekCameraViewer",
    "type_name",         "creekCameraViewer",
    "description",       "creekCameraViewer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekCameraViewer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekCameraViewer::creekCameraViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_creekCameraViewerServicePort("creekCameraViewerService")
{
}

creekCameraViewer::~creekCameraViewer()
{
}


RTC::ReturnCode_t creekCameraViewer::onInitialize()
{
  std::cout << "creekCameraViewer : onInitialize" << std::endl;

  m_creekCameraViewerServicePort.registerProvider("service0", "creekCameraViewerService", m_service0);
  addPort(m_creekCameraViewerServicePort);


  RTC::Properties& prop = getProperties();
  coil::vstring camera_name_list = coil::split( prop["CAMERA_NAME_LIST"], ",");
  m_ports.resize(camera_name_list.size());
  for(int i=0; i<camera_name_list.size(); i++) {
    m_ports[i] = new CameraPort(camera_name_list[i].c_str());
    addInPort(camera_name_list[i].c_str(), *m_ports[i]);
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekCameraViewer : onActivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekCameraViewer : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onExecute(RTC::UniqueId ec_id)
{
  for(int i=0; i<m_ports.size(); i++) {
    if( m_ports[i]->isNew() ) {
      m_ports[i]->read();
      if( m_dec.detectQrCode(m_ports[i]->m_frame) && m_service0.draw() ) {
	m_dec.drawFinderPattern(m_ports[i]->m_frame);
	m_dec.drawVertices(m_ports[i]->m_frame);
      }
    }
  }

  combineImage();
  cv::imshow("all", m_allImage);
  //cv::waitKey(1);

  return RTC::RTC_OK;
}


void creekCameraViewer::combineImage()
{
  int n = m_ports.size();
  int nx = ceil( sqrt(n) );
  int ny = ceil( double(n)/double(nx) );

  int frameX = m_ports[0]->m_frame.cols;
  int frameY = m_ports[0]->m_frame.rows;
  m_allImage = cv::Mat::zeros( cv::Size(frameX*nx + (nx-1), frameY*ny + (ny-1) ), CV_8UC3);

  //std::cout << "all image" << m_allImage.cols << "x" << m_allImage.rows << std::endl;
  cv::Rect roi_rect;
  int i=0;
  for(int y=0; y<ny; y++) {
    for(int x=0; x<nx; x++) {
      if( i < n ) {
	roi_rect.width  = m_ports[i]->m_frame.cols;
	roi_rect.height = m_ports[i]->m_frame.rows;
	cv::Mat roi(m_allImage, roi_rect);
	m_ports[i]->m_frame.copyTo(roi);
	roi_rect.x += m_ports[i]->m_frame.cols + 1;
      }
      i++;
    }
    if( i < n ) {
      roi_rect.x  = 0;
      roi_rect.y += m_ports[i]->m_frame.rows + 1;
    }
  }
}


extern "C"
{
 
  void creekCameraViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekcameraviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekCameraViewer>,
                             RTC::Delete<creekCameraViewer>);
  }
  
};



