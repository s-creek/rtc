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


template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}


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
  int n = camera_name_list.size();
  m_ports.resize(n);
  for(int i=0; i<n; i++) {
    m_ports[i] = new CameraPort(camera_name_list[i].c_str());
    addInPort(camera_name_list[i].c_str(), *m_ports[i]);
  }


  m_cameraPose.resize(n);
  m_cameraPoseIn.resize(n);
  for(int i=0; i<n; i++) {
    std::string name = camera_name_list[i] + "Pose";
    m_cameraPoseIn[i] = new InPort<TimedPose3D>(name.c_str(), m_cameraPose[i]);
    addInPort(name.c_str(), *m_cameraPoseIn[i]);
  }


  m_maxSeqNum = 35;
  m_tcsSeq.resize(n);


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
  // get camera pose
  for(int i=0; i<m_cameraPoseIn.size(); i++) {
    if( m_cameraPoseIn[i]->isNew() ) {
      m_cameraPoseIn[i]->read();

      TimedCoordinateSystem tmp;
      tmp.tm = toSec(m_cameraPose[i].tm);
      tmp.p << m_cameraPose[i].data.position.x, m_cameraPose[i].data.position.y, m_cameraPose[i].data.position.z;
      tmp.R = cnoid::rotFromRpy(m_cameraPose[i].data.orientation.r, m_cameraPose[i].data.orientation.p, m_cameraPose[i].data.orientation.y);

      // logging
      m_tcsSeq[i].push_back(tmp);
      if( m_tcsSeq[i].size() > m_maxSeqNum ) {
	m_tcsSeq[i].pop_front();
      }

      //std::cout << m_cameraPoseIn[i]->name() << " : time = " << tmp.tm << ",  pos = " << tmp.p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
    }
  }


  // get image data
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


//
//  y              +---------x
//  |              |
//  | cnoid        |  OpenCV
//  |              |
//  +---------x    y
//
cnoid::Vector3 creekCameraViewer::pixelToVector(double x, double y)
{
  double width(640), height(480), fieldOfView(1.0);

  double dist = height / 2.0 / tan(fieldOfView/2.0);
  double cx = (width-1) / 2.0;   // center x
  double cy = (height-1) / 2.0;  // center y

  double vx =  x - cx;
  double vy = -y + cy;

  cnoid::Vector3 vec(vx, vy, -dist);
  vec.normalize();
  return vec;
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



