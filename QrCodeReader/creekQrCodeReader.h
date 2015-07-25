// -*- C++ -*-
/*!
 * @file  creekQrCodeReader.h * @brief creekQrCodeReader * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef CREEKQRCODEREADER_H
#define CREEKQRCODEREADER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <cnoid/corba/CameraImage.hh>
#include <opencv2/opencv.hpp>
#include <zbar.h>

// Service implementation headers
#include "creekQrCodeReaderService_impl.h"

using namespace RTC;

class creekQrCodeReader  : public RTC::DataFlowComponentBase
{
public:
  creekQrCodeReader(RTC::Manager* manager);
  ~creekQrCodeReader();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected:
  Img::TimedCameraImage m_image;
  InPort<Img::TimedCameraImage> m_imageIn;

  RTC::CorbaPort m_creekQrCodeReaderServicePort;
  creekQrCodeReaderService_impl m_service0;


private:
  bool updateImage();

  cv::Mat m_frame, m_gray;
  zbar::Image m_zbar;
  zbar::ImageScanner m_scanner;
};


extern "C"
{
  DLL_EXPORT void creekQrCodeReaderInit(RTC::Manager* manager);
};

#endif // CREEKQRCODEREADER_H

