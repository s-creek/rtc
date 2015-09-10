// -*- C++ -*-

#ifndef CREEKGAITPATTERNGENERATORCP_H
#define CREEKGAITPATTERNGENERATORCP_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekGaitPatternGeneratorCpService_impl.h"

#include <WalkPlanner.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

using namespace RTC;

class creekGaitPatternGeneratorCp  : public RTC::DataFlowComponentBase
{
public:
  creekGaitPatternGeneratorCp(RTC::Manager* manager);
  ~creekGaitPatternGeneratorCp();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void startStepping();
  void stopStepping();
  void test();


protected:
  TimedDoubleSeq m_qInit;
  InPort<TimedDoubleSeq> m_qInitIn;
  TimedPoint3D m_basePosInit;
  InPort<TimedPoint3D> m_basePosInitIn;
  TimedOrientation3D m_baseRpyInit;
  InPort<TimedOrientation3D> m_baseRpyInitIn;
  TimedPoint3D m_zmpRefInit;
  InPort<TimedPoint3D> m_zmpRefInitIn;


  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedPoint3D m_basePos;
  OutPort<TimedPoint3D> m_basePosOut;
  TimedOrientation3D m_baseRpy;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedPoint3D m_zmpRef;
  OutPort<TimedPoint3D> m_zmpRefOut;

  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;


  RTC::CorbaPort m_creekGaitPatternGeneratorCpServicePort;
  creekGaitPatternGeneratorCpService_impl m_service0;


private:
  void initPlanner();

  WalkPlanner m_planner;
  RobotPtr    m_robot;

  creek::Matrix3  m_waistRef;
  creek::Position m_rfootRef, m_lfootRef;

  boost::mutex mtx;
};


extern "C"
{
  DLL_EXPORT void creekGaitPatternGeneratorCpInit(RTC::Manager* manager);
};

#endif // CREEKGAITPATTERNGENERATORCP_H

