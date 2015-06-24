// -*- C++ -*-
/*!
 * @file  ArmControlCartesian.h * @brief Sequence InPort component * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef ARMCONTROLCARTESIAN_H
#define ARMCONTROLCARTESIAN_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "HrprtcBase.h"

// Service implementation headers
#include "ArmControlCartesianService_impl.h"

// user library
//#include <interpolationHeaders.h>
#include <Interpolator.h>
#include <RotationalInterpolator.h>


using namespace RTC;

class ArmControlCartesian  : public HrprtcBase
{
 public:
  ArmControlCartesian(RTC::Manager* manager);
  ~ArmControlCartesian();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  
  // service port  
  bool isEmpty();
  bool setTargetAngular(double x, double y, double z, double r, double p, double w, double time);
  void getCurrentConfiguration(double &x, double &y, double &z, double &r, double &p, double &w);
  void emergencyStop();


 protected:
  // CORBA Port declaration
  RTC::CorbaPort m_ArmControlCartesianServicePort;

  // Service declaration
  ArmControlCartesianService_impl m_service0;


 private:
  int  m_assignedPartIndex;
  hrp::JointPathPtr m_jointPath;

  creek::Interpolator * m_pos_interpolator;
  creek::RotationalInterpolator * m_rot_interpolator;
};


extern "C"
{
  DLL_EXPORT void ArmControlCartesianInit(RTC::Manager* manager);
};

#endif // ARMCONTROLCARTESIAN_H

