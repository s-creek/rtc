// -*- C++ -*-

#ifndef CREEKSEQUENCEPLAYER_H
#define CREEKSEQUENCEPLAYER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekSequencePlayerService_impl.h"

#include <cnoid/Body>
#include "VectorConvert.h"

#include <creekInterpolator/Interpolator.h>

using namespace RTC;

class creekSequencePlayer  : public RTC::DataFlowComponentBase
{
public:
  creekSequencePlayer(RTC::Manager* manager);
  ~creekSequencePlayer();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


  inline unsigned int dof() const { return m_dof; };

  bool setJointAngles(const double *angles, double tm);
  bool isEmpty();

protected:
  TimedDoubleSeq m_qInit;
  InPort<TimedDoubleSeq> m_qInitIn;

  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;

  RTC::CorbaPort m_creekSequencePlayerServicePort;
  creekSequencePlayerService_impl m_service0;


private:
  double m_dt;
  unsigned int m_dof;
  cnoid::BodyPtr m_robot;

  creek::Interpolator *m_seqAngles;
  creek::Interpolator *m_seqBasePos;
  creek::Interpolator *m_seqBaseRpy;
  creek::Interpolator *m_seqZmp;


  void calcCoM();
};


extern "C"
{
  DLL_EXPORT void creekSequencePlayerInit(RTC::Manager* manager);
};

#endif // CREEKSEQUENCEPLAYER_H

