// -*- c++ -*-

#ifndef SINGLE_IMAGE_PROCESSING_H
#define SINGLE_IMAGE_PROCESSING_H

// rtm
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// user
#include "ImageInterfaceTypes.h"


class SingleImageProcessing : public RTC::DataFlowComponentBase
{
 public:
  SingleImageProcessing(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  RTC::InPort<RTC::TimedLongSeq> *m_imageIn;
  ImageInterfaceTypes *m_image;


  RTC::OutPort<RTC::TimedDoubleSeq> m_targetConfOut;
  RTC::TimedDoubleSeq m_targetConf;

private:
  std::string m_instanceName;
};


extern "C"
{
  DLL_EXPORT void SingleImageProcessingInit(RTC::Manager* manager);
};


#endif
