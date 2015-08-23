// -*- C++ -*-

#include "creekPointCloudViewer.h"

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

static const char* creekpointcloudviewer_spec[] =
  {
    "implementation_id", "creekPointCloudViewer",
    "type_name",         "creekPointCloudViewer",
    "description",       "creekPointCloudViewer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekPointCloudViewer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}

double toDeg(double rad)
{
  return rad / M_PI * 180.0;
}


creekPointCloudViewer::creekPointCloudViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_rangerIn("ranger", m_ranger),
    m_qCurIn("qCur", m_qCur),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_creekPointCloudViewerServicePort("creekPointCloudViewerService"),
    m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    //m_viewer(new pcl::visualization::CloudViewer("Cloud Viewer"))
    //m_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"))
    m_immediately(true),
    m_rfootT(vtkSmartPointer<vtkTransform>::New()),
    m_lfootT(vtkSmartPointer<vtkTransform>::New())
{
  m_service0.setComponent(this);
}


creekPointCloudViewer::~creekPointCloudViewer()
{
}


RTC::ReturnCode_t creekPointCloudViewer::onInitialize()
{
  // setup port
  addInPort("ranger", m_rangerIn);
  addInPort("qCur", m_qCurIn);
  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);

  m_creekPointCloudViewerServicePort.registerProvider("service0", "creekPointCloudViewerService", m_service0);
  addPort(m_creekPointCloudViewerServicePort);


  // setup model
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str() );
  
  m_sensor = m_robot->findDevice<cnoid::RangeSensor>( prop["RANGE_SENSOR_NAME"] );
  m_rfoot = m_robot->link(prop["RLEG_END"]);
  m_lfoot = m_robot->link(prop["LLEG_END"]);
  coil::stringTo(m_ankleHeight, prop["ankle_height"].c_str());

  std::cout << "creekPointCloudViewer : robot name = " << m_robot->name() << std::endl;
  std::cout << "creekPointCloudViewer : sesor name = " << m_sensor->name() << std::endl;


  // setup pcl (cloud)
  m_cloud->reserve(500000);
  

  // max buffer size
  double dt;
  coil::stringTo(dt, prop["pdservo.dt"].c_str());
  m_maxSeqNum = 5 / dt / m_sensor->frameRate();
  if( m_maxSeqNum < 10 )
    m_maxSeqNum = 10;


  std::cout << "creekPointCloudViewer : onInitialize" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onActivated(RTC::UniqueId ec_id)
{
  m_active = false;


  // init data port
  m_basePos.data.x = 0.0;
  m_basePos.data.y = 0.0;
  m_basePos.data.z = 0.0;

  m_baseRpy.data.r = 0.0;
  m_baseRpy.data.p = 0.0;
  m_baseRpy.data.y = 0.0;


  // log for range sensor coordinate system
  m_tcsSeq.clear();
 

  // setup pcl (viewer)
  if( !m_viewer ) {
    m_viewer.reset(new pcl::visualization::PCLVisualizer("Cloud Viewer")); 
    m_viewer->addPointCloud(m_cloud, "cloud");
    m_viewer->initCameraParameters();
    m_viewer->setSize(1280, 720);
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->addCoordinateSystem(0.1);
    m_viewer->setCameraPosition(-10, 0, 3,
				0, 0, 1.5,
				0, 0, 1);

    // test
    if( false ) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor(new pcl::PointCloud<pcl::PointXYZRGB>);
      for(int i=-50; i<=50; i++) {
	for(int j=-50; j<=50; j++) {
	  pcl::PointXYZRGB point;
	  point.x = i*0.1;
	  point.y = j*0.1;
	  point.z = 0.0;
	  point.b = 255;
	  point.g = 120;
	  floor->push_back(point);
	}
      }
      m_viewer->addPointCloud(floor, "floor");
    }

    RTC::Properties& prop = getProperties();
    m_viewer->addModelFromPLYFile(prop["RFOOT_MODEL_NAME"], m_rfootT, "rfoot");
    m_viewer->addModelFromPLYFile(prop["LFOOT_MODEL_NAME"], m_lfootT, "lfoot");
  }


  std::cout << "creekPointCloudViewer : onActivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onDeactivated(RTC::UniqueId ec_id)
{
  m_active = false;
  m_tcsSeq.clear();

  if( m_viewer ) {
    //m_viewer->close();
    m_viewer.reset();
  }


  std::cout << "creekPointCloudViewer : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onExecute(RTC::UniqueId ec_id)
{
  if(m_basePosIn.isNew()) m_basePosIn.read();
  if(m_baseRpyIn.isNew()) m_baseRpyIn.read();


  if(m_qCurIn.isNew()) {
    m_qCurIn.read();

    // update model
    for(int i=0; i<m_robot->numJoints(); i++) {
      m_robot->joint(i)->q() = m_qCur.data[i];
    }
    m_robot->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
    m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    m_robot->calcForwardKinematics();

    // calc sensor p,R (on world)
    TimedCoordinateSystem tmp;
    tmp.tm = toSec(m_qCur.tm);
    tmp.p  = m_sensor->link()->p() + m_sensor->link()->R() * m_sensor->p_local();
    tmp.R  = m_sensor->link()->R() * m_sensor->R_local();

    // logging
    m_tcsSeq.push_back(tmp);
    if( m_tcsSeq.size() > m_maxSeqNum ) {
      m_tcsSeq.pop_front();
    }
  }


  if(m_rangerIn.isNew()) {
    m_rangerIn.read();
    //std::cout << "range : " << toSec(m_ranger.tm) << ",  q : " << toSec(m_qCur.tm) << std::endl;

    if(m_active) {
      cnoid::Vector3 sp;
      cnoid::Matrix3 sR;


      // detect coordinate system
      bool detect(false);
      double tm = toSec(m_ranger.tm);
      for( std::deque<TimedCoordinateSystem>::reverse_iterator it = m_tcsSeq.rbegin(); it != m_tcsSeq.rend(); it++) {
	if( it->tm <= tm ) {
	  sp = it->p;
	  sR = it->R;
	  m_tcsSeq.erase(m_tcsSeq.begin(), it.base());
	  detect = true;
	  break;
	}
      }
      if( !detect )
	return RTC::RTC_OK;


      // calc point (on world)
      int num = m_ranger.ranges.length();
      double *data    = m_ranger.ranges.get_buffer();
      double step     = m_ranger.config.angularRes;
      double minAngle = m_ranger.config.minAngle;
      double minRange = m_ranger.config.minRange;
      double maxRange = m_ranger.config.maxRange;
      for(int i=0; i<num; i++) {
	if( data[i] >= minRange && data[i] <= maxRange) {
	  double angle = minAngle + i*step;
	  cnoid::Vector3 p_local( data[i]*sin(-angle), 0.0, -data[i]*cos(angle));
	  cnoid::Vector3 p_world = sp + sR * p_local;

	  pcl::PointXYZRGB point;
	  point.x = p_world[0];
	  point.y = p_world[1];
	  point.z = p_world[2];
	  point.r = 255;
	  point.g = 255;
	  point.b = 255;
	  m_cloud->push_back(point);
	}
      }
      if( m_immediately ) {
	//m_viewer->showCloud(m_cloud->makeShared());
	m_viewer->updatePointCloud(m_cloud, "cloud");
      }
    }
  }
  m_viewer->spinOnce();


  return RTC::RTC_OK;
}


void creekPointCloudViewer::start()
{
  m_active = true;
  m_cloud->clear();


  { // RFOOT
    cnoid::Vector3 pos = m_rfoot->p();
    cnoid::Vector3 rpy = m_rfoot->R().eulerAngles(2,1,0);
    m_rfootT->Identity();  // reset
    m_rfootT->Translate(pos[0], pos[1], pos[2]);
    m_rfootT->RotateZ(rpy[0]/M_PI*180.0);
    m_rfootT->RotateY(rpy[1]/M_PI*180.0);
    m_rfootT->RotateX(rpy[2]/M_PI*180.0);
  }
  { // LFOOT
    cnoid::Vector3 pos = m_lfoot->p();
    cnoid::Vector3 rpy = m_lfoot->R().eulerAngles(2,1,0);
    m_lfootT->Identity();  // reset
    m_lfootT->Translate(pos[0], pos[1], pos[2]);
    m_lfootT->RotateZ(rpy[0]/M_PI*180.0);
    m_lfootT->RotateY(rpy[1]/M_PI*180.0);
    m_lfootT->RotateX(rpy[2]/M_PI*180.0);
  }
}


void creekPointCloudViewer::stop()
{
  m_active = false;

  // downsampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr org(new pcl::PointCloud<pcl::PointXYZRGB>());
  org = m_cloud->makeShared();  // deep copy
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
  filter.setInputCloud(org);
  filter.setLeafSize (0.01, 0.01, 0.01);  // 10mm
  filter.filter(*m_cloud);

  //m_viewer->showCloud(m_cloud->makeShared());
  m_viewer->updatePointCloud(m_cloud, "cloud");
  std::cout << "creekPointCloudViewer : cloud size = " << org->size() << std::endl;
  std::cout << "creekPointCloudViewer : cloud size = " << m_cloud->size() << std::endl;
}


bool creekPointCloudViewer::detectLandingPoint(double x, double y, double w, int ft)
{
  double range(0.2), minRange(0.05);
  double bankLimit(30.0); // degree
  int minReqNum(10);


  // set detect area
  std::vector<int> indices;
  indices.reserve(m_cloud->size());
  for( int i=0; i<m_cloud->size(); i++) {
    cnoid::Vector3 dist;
    dist << (m_cloud->points[i].x-x), (m_cloud->points[i].y-y), 0.0;
    if( dist.norm() < range ) {
      indices.push_back(i);
      m_cloud->points[i].r = 0;
      m_cloud->points[i].g = 0;
      m_cloud->points[i].b = 255;
    }
  }
  if( indices.size() < minReqNum ) {
    std::cout << "creekPointCloudViewer : too little of point (set area)" << std::endl;
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr area(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*m_cloud, indices, *area);


  // detect plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1);
  
  seg.setInputCloud(area->makeShared ());
  seg.segment(*inliers, *coefficients);

  if( inliers->indices.size() < minReqNum ) {
    std::cout << "creekPointCloudViewer : too little of point (detect plane)" << std::endl;
    return false;
  }
  for(int i=0; i<inliers->indices.size(); i++) {
    int id = indices[inliers->indices[i]];
    m_cloud->points[id].r = 255;
    m_cloud->points[id].g = 0;
    m_cloud->points[id].b = 0;
  }


  // calc landing point
  cnoid::Vector3 ez(0,0,1);
  cnoid::Vector3 n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  n.normalize();
  if( n[2] < 0 )
    n = -n;
  if( n.dot(ez) < cos(bankLimit/180.0*M_PI) ) {
    std::cout << "creekPointCloudViewer : bank limit over" << std::endl;
    //return false;
  }
  // plane equation : ax+ by+ cz + d = 0
  // coefficients values : a,b,c,d
  double z = -(coefficients->values[0]*x + coefficients->values[1]*y + coefficients->values[3]) / coefficients->values[2];

  Eigen::Vector4f center; 
  pcl::compute3DCentroid(*area, inliers->indices, center);
  cnoid::Vector3 dist(x-center[0], y-center[1], z-center[2]);
  if( dist.norm() < minRange ) {
    x = center[0];
    y = center[1];
    z = center[2];
    std::cout << "creekPointCloudViewer : move landing point" << std::endl;
  }


  // calc mode position and orientation
  cnoid::Vector3 pos(x,y,z);
  pos += (m_ankleHeight * n);

  cnoid::Vector3 axis;
  axis = ez.cross(n);
  double angle = acos(ez.dot(n)) / M_PI * 180.0;

  std::cout << "creekPointCloudViewer : landing point" << std::endl;
  std::cout << "  pos = " << pos[0] << "  " << pos[1] << "  " << pos[2] << std::endl;
  std::cout << "  n = " << n[0] << "  " << n[1] << "  " << n[2] << std::endl;


  // set model
  if( ft == 0 ) {
    m_rfootT->Identity();  // reset
    m_rfootT->Translate(pos[0], pos[1], pos[2]);
    m_rfootT->RotateWXYZ(angle, axis[0], axis[1], axis[2]);
    m_rfootT->RotateZ(w/M_PI*180.0);
  }
  else if(ft == 1) {
    m_lfootT->Identity();  // reset
    m_lfootT->Translate(pos[0], pos[1], pos[2]);
    m_lfootT->RotateWXYZ(angle, axis[0], axis[1], axis[2]);
    m_lfootT->RotateZ(w/M_PI*180.0);
  }
  else {
    return false;
  }
  m_viewer->updatePointCloud(m_cloud, "cloud");
  return true;
}


void creekPointCloudViewer::test()
{
  std::cout << "creekPointCloudViewer : test" << std::endl;

  if(true) {
    std::cout  << "creekPointCloudViewer" << std::endl;
    std::cout << "  time = " << toSec(m_ranger.tm) << std::endl;
    std::cout << "  size = " << m_ranger.ranges.length() << std::endl;
    std::cout << "  config" << std::endl;
    std::cout << "      minAngle    = " << toDeg(m_ranger.config.minAngle) << std::endl;
    std::cout << "      maxAngle    = " << toDeg(m_ranger.config.maxAngle) << std::endl;
    std::cout << "      angularRes  = " << toDeg(m_ranger.config.angularRes) << std::endl;
    std::cout << "      minRange    = " << m_ranger.config.minRange << std::endl;
    std::cout << "      maxRange    = " << m_ranger.config.maxRange << std::endl;
    std::cout << "      rangeRes    = " << m_ranger.config.rangeRes << std::endl;
    std::cout << "      frequency   = " << m_ranger.config.frequency << std::endl;
    std::cout << "  geometry" << std::endl;
    std::cout << "      [ x, y, z ] = [ " << m_ranger.geometry.geometry.pose.position.x << ", " << m_ranger.geometry.geometry.pose.position.y << ", " << m_ranger.geometry.geometry.pose.position.z << " ]" << std::endl;
    std::cout << "      [ r, p, y ] = [ " << m_ranger.geometry.geometry.pose.orientation.r << ", " << m_ranger.geometry.geometry.pose.orientation.p << ", " << m_ranger.geometry.geometry.pose.orientation.y << " ]" << std::endl;
    std::cout << "      [ w, l, h ] = [ " << m_ranger.geometry.geometry.size.w << ", " << m_ranger.geometry.geometry.size.l << ", " << m_ranger.geometry.geometry.size.h << " ]" << std::endl;
  }
}



extern "C"
{
 
  void creekPointCloudViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekpointcloudviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekPointCloudViewer>,
                             RTC::Delete<creekPointCloudViewer>);
  }
  
};



