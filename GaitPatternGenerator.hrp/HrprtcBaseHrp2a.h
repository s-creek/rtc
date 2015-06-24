// -*- C++ -*-

#ifndef HRPRTCBASE_HRP2A_H
#define HRPRTCBASE_HRP2A_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <hrpModel/Body.h>
#include "VectorConvert.h"


using namespace RTC;


class HrprtcBaseHrp2a : public RTC::DataFlowComponentBase
{
public:
  HrprtcBaseHrp2a(RTC::Manager* manager);
  ~HrprtcBaseHrp2a();

  virtual RTC::ReturnCode_t onInitialize();


protected:
  //TimedDoubleSeq m_qInit;
  //InPort<TimedDoubleSeq> m_qInitIn;
  TimedDoubleSeq m_jointDat;
  InPort<TimedDoubleSeq> m_jointDatIn;
  OutPort<TimedDoubleSeq> m_jointDatOut;
  

  TimedDoubleSeq m_basePosInit;
  InPort<TimedDoubleSeq> m_basePosInitIn;
  TimedDoubleSeq m_baseRpyInit;
  InPort<TimedDoubleSeq> m_baseRpyInitIn;


  TimedDoubleSeq m_zmpRef;
  OutPort<TimedDoubleSeq> m_zmpRefOut;
  TimedDoubleSeq m_accRef;
  OutPort<TimedDoubleSeq> m_accRefOut;
  TimedDoubleSeq m_basePos;
  OutPort<TimedDoubleSeq> m_basePosOut;
  TimedDoubleSeq m_baseRpy;
  OutPort<TimedDoubleSeq> m_baseRpyOut;

  
public:
  double m_dt;
  std::string m_instanceName;

  hrp::BodyPtr m_robot;
};

#endif // HRPRTCBASE_HRP2A_H
