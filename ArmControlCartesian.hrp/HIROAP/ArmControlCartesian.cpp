// -*- C++ -*-
/*!
 * @file  ArmControlCartesian.cpp * @brief Sequence InPort component * $Date$ 
 *
 * $Id$ 
 */
#include "ArmControlCartesian.h"

#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>

// Module specification
static const char* armcontrolcartesian_spec[] =
  {
    "implementation_id", "ArmControlCartesian",
    "type_name",         "ArmControlCartesian",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "skj",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };

ArmControlCartesian::ArmControlCartesian(RTC::Manager* manager)
  : HrprtcBase(manager),
    m_ArmControlCartesianServicePort("ArmControlCartesianService")
{
  m_service0.setComponent(this);
}

ArmControlCartesian::~ArmControlCartesian()
{
}


RTC::ReturnCode_t ArmControlCartesian::onInitialize()
{
  // set properties
  HrprtcBase::onInitialize();


  //
  // Registration: InPort/OutPort/Service
  //
  // Set service provider to Ports
  m_ArmControlCartesianServicePort.registerProvider("service0", "ArmControlCartesianService", m_service0);

  // Set CORBA Service Ports
  addPort(m_ArmControlCartesianServicePort);


  //
  // set using part
  //
  // get properties
  RTC::Properties& prop = getProperties();
  std::string assignedPartName = prop["part.assigned_part."+m_instanceName];
  m_assignedPartIndex = -1;
  for(int i = 0; i < m_partNum; i++) {
    if ( m_partNames[i] == assignedPartName )
      m_assignedPartIndex = i;
  }
  if ( m_assignedPartIndex < 0 ) {
    std::cerr << m_instanceName << " : No part assigned Error." << std::endl;
    return RTC::RTC_ERROR;
  }
  std::cout << m_instanceName << " : using " << assignedPartName << " (part id=" << m_assignedPartIndex << ")" << std::endl;


  //
  // initialize jointPath
  //
  hrp::Link * first = m_robot->joint (*(m_partMembers[m_assignedPartIndex].begin ()));
  hrp::Link * base = m_robot->joint (first->parent->jointId);
  hrp::Link * end = m_robot->joint (*(m_partMembers[m_assignedPartIndex].end () - 1));
  m_jointPath = m_robot->getJointPath (base, end);
  m_jointPath->setBestEffortIKMode(true);
  std::cout << m_instanceName << " : " << assignedPartName << " member list =";
  for(int i = 0; i < m_jointPath->numJoints(); i++)
    std::cout << "  " << m_jointPath->joint(i)->name;
  std::cout << std::endl;

  //
  // initialize interpolator
  //
  m_pos_interpolator = new creek::Interpolator(3, m_dt, creek::QUINTIC);
  m_rot_interpolator = new creek::RotationalInterpolator(m_dt, creek::QUINTIC);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmControlCartesian::onExecute(RTC::UniqueId ec_id)
{
  //
  // asigned part index
  //
  int aid = m_assignedPartIndex;


  //
  // get data from in port & update robot model
  //
  if(m_jointDatIn.isNew ()) {
    m_jointDatIn.read();
    for(unsigned int i = 0; i < m_partMembers.size (); i++) {
      for(unsigned int j = 0; j < m_partMembers[i].size (); j++) {
	hrp::Link * l = m_robot->joint((int) m_partMembers[i][j]);
	if(l != NULL)
	  l->q = m_jointDat.qState[i][j];
      }
    }
    m_robot->calcForwardKinematics ();
  }
  else
    return RTC::RTC_OK;


  //
  // check reference data
  //
  if( isEmpty() )
    return RTC::RTC_OK;


  //
  // set data for not asigined part
  //
  for(int i=0; i<m_jointDat.id.length(); i++) {
    if(i != aid) {
      m_jointDat.id[i] = CMD_IDLE;
      m_jointDat.cmd[i].sec = 0;
      m_jointDat.cmd[i].nsec = 0;
    }
  }


  //
  // clear motion
  //
  if (  m_jointDat.id[aid] == CMD_GO_ACTUAL
	|| m_jointDat.id[aid] == CMD_PROTECTIVE_STOP
	|| m_cmdTimes[aid].sec  <  m_jointDat.cmd[aid].sec
	|| (m_cmdTimes[aid].sec  == m_jointDat.cmd[aid].sec &&
	    m_cmdTimes[aid].nsec <  m_jointDat.cmd[aid].nsec))
    {
      emergencyStop();
      m_jointDat.id[aid] = CMD_PROTECTIVE_STOP;
      m_jointDat.cmd[aid].sec  = m_cmdTimes[aid].sec;
      m_jointDat.cmd[aid].nsec = m_cmdTimes[aid].nsec;
      m_jointDatOut.write ();
      return RTC::RTC_OK;
    }

  //
  // write to out port
  //
  else
    {
      // calc inverse kinematics
      hrp::Vector3  ref_p;   m_pos_interpolator->get(&ref_p[0]);
      hrp::Matrix33 ref_R0;  m_rot_interpolator->get(ref_R0);
      hrp::Matrix33 ref_R1 = m_jointPath->endLink()->calcRfromAttitude(ref_R0);
      if( !m_jointPath->calcInverseKinematics(ref_p, ref_R1) ) {
	emergencyStop();
	return RTC::RTC_OK;
      }

      // update port dat
      for(int i = 0; i < m_jointPath->numJoints(); i++)
	m_jointDat.qCommand[aid][i] = m_jointPath->joint(i)->q;

      m_jointDat.id[aid]       = m_cmdId;
      m_jointDat.cmd[aid].sec  = m_cmdTimes[aid].sec;
      m_jointDat.cmd[aid].nsec = m_cmdTimes[aid].nsec;

      setCurrentTime (m_jointDat.tm);
      m_jointDatOut.write ();
      return RTC::RTC_OK;
    }


  return RTC::RTC_OK;
}


bool ArmControlCartesian::isEmpty()
{
  return (m_pos_interpolator->empty() || m_rot_interpolator->empty());
}


bool ArmControlCartesian::setTargetAngular(double x, double y, double z, double r, double p, double w, double time)
{
  if( !isEmpty() )
    return false;


  //
  // set command time
  //
  setCmdTime(m_assignedPartIndex);


  //
  // position
  //
  hrp::Vector3 p_start(m_jointPath->endLink()->p);
  hrp::Vector3 p_end(x,y,z);

  if( !m_pos_interpolator->calcInterpolation(&p_start(0), &p_end(0), time) ) {
    std::cout << m_instanceName << " : calc pos interpolation error" << std::endl;
    return false;
  }


  //
  // attitude
  //
  hrp::Matrix33 R_start(m_jointPath->endLink()->attitude());
  hrp::Matrix33 R_end(hrp::rotFromRpy(r,p,w));

  if( !m_rot_interpolator->calcInterpolation(R_start, R_end, hrp::Vector3(0,0,-1), time) ) {
    m_pos_interpolator->clear();
    std::cout << m_instanceName << " : calc rot interpolation error" << std::endl;
    return false;
  }

  //std::cout <<  m_instanceName << " : reference attitude = " << R_end << std::endl;

  return true;
}


void ArmControlCartesian::getCurrentConfiguration(double &x, double &y, double &z, double &r, double &p, double &w)
{
  x = m_jointPath->endLink ()->p[0];
  y = m_jointPath->endLink ()->p[1];
  z = m_jointPath->endLink ()->p[2];

  hrp::Vector3 rpy = hrp::rpyFromRot(m_jointPath->endLink()->attitude());
  r = rpy[0];
  p = rpy[1];
  w = rpy[2];

  //std::cout <<  m_instanceName << " : current attitude = " << m_jointPath->endLink()->attitude() << std::endl;
}


void ArmControlCartesian::emergencyStop()
{
  m_pos_interpolator->clear();
  m_rot_interpolator->clear();
}



//-----------------------------------------------------------------------

extern "C"
{
 
  void ArmControlCartesianInit(RTC::Manager* manager)
  {
    coil::Properties profile(armcontrolcartesian_spec);
    manager->registerFactory(profile,
                             RTC::Create<ArmControlCartesian>,
                             RTC::Delete<ArmControlCartesian>);
  }
  
};
