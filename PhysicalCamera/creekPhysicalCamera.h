// -*- C++ -*-
/*!
 * @file  creekPhysicalCamera.h * @brief creekPhysicalCamera * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef CREEKPHYSICALCAMERA_H
#define CREEKPHYSICALCAMERA_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <cnoid/corba/CameraImage.hh>
#include <opencv2/opencv.hpp>

#include "creekPhysicalCameraService_impl.h"

using namespace RTC;

class creekPhysicalCamera  : public RTC::DataFlowComponentBase
{
public:
  creekPhysicalCamera(RTC::Manager* manager);
  ~creekPhysicalCamera();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void show();

protected:
  Img::TimedCameraImage m_image;
  OutPort<Img::TimedCameraImage> m_imageOut;

  RTC::CorbaPort m_creekPhysicalCameraServicePort;
  creekPhysicalCameraService_impl m_service0;

private:
  cv::VideoCapture m_camera;
  cv::Mat m_frame;

  bool m_show;
};


extern "C"
{
  DLL_EXPORT void creekPhysicalCameraInit(RTC::Manager* manager);
};

#endif // CREEKPHYSICALCAMERA_H

