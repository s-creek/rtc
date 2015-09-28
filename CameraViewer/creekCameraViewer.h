// -*- C++ -*-

#ifndef CREEKCAMERAVIEWER_H
#define CREEKCAMERAVIEWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
#include "creekCameraViewerService_impl.h"

#include "CameraPort.h"
#include <creekQrCodeDetector.h>
#include <cnoid/EigenUtil>

using namespace RTC;

class creekCameraViewer  : public RTC::DataFlowComponentBase
{
public:
  creekCameraViewer(RTC::Manager* manager);
  ~creekCameraViewer();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected:
  std::vector< CameraPort * > m_ports;

  std::vector<TimedPose3D>             m_cameraPose;
  std::vector< InPort<TimedPose3D> * > m_cameraPoseIn;

  RTC::CorbaPort m_creekCameraViewerServicePort;
  creekCameraViewerService_impl m_service0;

private:
  cnoid::Vector3 pixelToVector(double x, double y);

  void combineImage();
  cv::Mat m_allImage;

  creek::creekQrCodeDetector m_dec;
  
  int m_maxSeqNum;
  struct TimedCoordinateSystem{
    double tm;
    cnoid::Vector3 p;
    cnoid::Matrix3 R;
  };
  std::vector< std::deque<TimedCoordinateSystem> > m_tcsSeq;
};


extern "C"
{
  DLL_EXPORT void creekCameraViewerInit(RTC::Manager* manager);
};

#endif // CREEKCAMERAVIEWER_H

