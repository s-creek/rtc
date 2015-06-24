// -*- c++ -*-

#ifndef GAIT_PATTERN_GENERATOR_H
#define GAIT_PATTERN_GENERATOR_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// rtc base
#include "HrprtcBaseHrp2a.h"

// Service implementation headers
#include "GaitPatternGeneratorService_impl.h"

// user library
//#include <Interpolator.h>
//#include <RotationalInterpolator.h>
#include <QueueInterpolator.h>
#include <FootPlanner.h>
#include <PreviewControl.h>

using namespace RTC;

class GaitPatternGenerator : public HrprtcBaseHrp2a
{
public:
  GaitPatternGenerator(RTC::Manager* manager);
  ~GaitPatternGenerator();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


  // service port
  RTC::CorbaPort m_GaitPatternGeneratorServicePort;
  GaitPatternGeneratorService_impl m_service0;

  void setTargetPos(double x, double y, double th, double time);
  void stop();
  
  
private:
  void updateModel();
  
  bool calcComInverseKinematics(const hrp::Vector3 &ref_com, const hrp::JointPathPtr legs, const hrp::Vector3 &ref_swing_foot_pos, const hrp::Matrix33 &ref_swing_foot_rot, const hrp::Matrix33 ref_waist_rot);
  
  creek::QueueInterpolator *m_line;
  creek::FootPlanner       *m_plan;
  creek::PreviewControl    *m_prev;

  hrp::JointPathPtr m_rleg, m_lleg, m_r2lleg, m_l2rleg;
  hrp::Vector3 m_init_rleg_pos, m_init_lleg_pos;
};


extern "C"
{
  DLL_EXPORT void GaitPatternGeneratorInit(RTC::Manager* manager);
};


#endif
