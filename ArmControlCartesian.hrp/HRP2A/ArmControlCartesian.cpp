// -*- C++ -*-
/*!
 * @file  ArmControlCartesian.cpp * @brief Sequence InPort component * $Date$ 
 *
 * $Id$ 
 */
#include "ArmControlCartesian.h"

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/ModelLoaderUtil.h>

//#define LOGGING
#ifdef LOGGING
#include <iostream>
#include <fstream>
#include <iostream>
std::ofstream userLog("/home/grxuser/public/ogawa/src/hrp/ArmControlCartesian.HRP2A/log/log.dat");
#endif


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
  : RTC::DataFlowComponentBase(manager),
    m_jointDatIn("jointDatIn", m_jointDat),
    m_jointDatOut("jointDatOut", m_jointDat),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_accRefOut("accRef", m_accRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_ArmControlCartesianServicePort("ArmControlCartesianService"),
    m_robot(NULL)
{
  m_service0.setComponent(this);
}

ArmControlCartesian::~ArmControlCartesian()
{
  m_pos_interpolator->clear();
  m_rot_interpolator->clear();

  #ifdef LOGGING
  userLog.close();
  #endif
}


RTC::ReturnCode_t ArmControlCartesian::onInitialize()
{
 //
  // Registration: InPort/OutPort/Service
  //
  // Set InPort buffers
  addInPort("jointDatIn", m_jointDatIn);
  addInPort("basePosInit", m_basePosInitIn);
  addInPort("baseRpyInit", m_baseRpyInitIn);

  // Set OutPort buffer
  addOutPort("jointDatOut", m_jointDatOut);
  addOutPort("zmpRef", m_zmpRefOut);
  addOutPort("accRef", m_accRefOut);
  addOutPort("basePos", m_basePosOut);
  addOutPort("baseRpy", m_baseRpyOut);

  // Set service provider to Ports
  m_ArmControlCartesianServicePort.registerProvider("service0", "ArmControlCartesianService", m_service0);
  
// Set CORBA Service Ports
  addPort(m_ArmControlCartesianServicePort);




  // get properties
  RTC::Properties& prop = getProperties();

  coil::stringTo(m_dt, prop["dt"].c_str());
  m_instanceName = prop["instance_name"];

  std::cout << m_instanceName << " : dt = " << m_dt << std::endl;



  //
  // get the reference of nameservice
  //
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int commaPos = nameServer.find(",");
  if (commaPos > 0)
    nameServer = nameServer.substr(0, commaPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  CosNaming::NamingContext_var m_rootNameContext = CosNaming::NamingContext::_duplicate(naming.getRootContext());


  //
  // setup robot model
  //
  m_robot = new hrp::Body();
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), m_rootNameContext))
    {
      std::cerr << m_instanceName << " : failed to load model[" << prop["model"] << "]" << std::endl;
    }


  //
  // set using part
  //
  // get properties
  //RTC::Properties& prop = getProperties();
  std::string assignedPartName = prop["part.assigned_part."+m_instanceName];
  std::cout << m_instanceName << " : using " << assignedPartName << std::endl;
  std::vector<int> m_partMembers;
  coil::stringTo(m_partMembers, prop["part.assigned_joint."+assignedPartName].c_str());
  int memberNum = m_partMembers.size();
  std::cout << m_instanceName << " : part." << assignedPartName << " has " << memberNum << " dof (";
  for (int j = 0; j < memberNum; j++)
    {
      std::cout << m_partMembers[j];
      if (j < memberNum - 1)
	std::cout << ",";
    }
  std::cout << ")" << std::endl;


  //
  // initialize jointPath
  //
  hrp::Link * first = m_robot->joint (*(m_partMembers.begin ()));
  hrp::Link * base = m_robot->joint (first->parent->jointId);
  hrp::Link * end = m_robot->joint (*(m_partMembers.end () - 1));
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


  //
  // set initial data
  //
  unsigned int dof = m_robot->numJoints();
  m_jointDat.data.length(dof);
  for(unsigned int i = 0; i < dof; i++) m_jointDat.data[i] = 0.0;

  m_basePosInit.data.length(3);
  m_baseRpyInit.data.length(3);
  for(int i = 0; i < 3; i++) {
    m_basePosInit.data[i] = 0.0;
    m_baseRpyInit.data[i] = 0.0;
  }


  m_zmpRef.data.length(3);
  m_accRef.data.length(3);
  m_basePos.data.length(3);
  m_baseRpy.data.length(3);


  return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmControlCartesian::onExecute(RTC::UniqueId ec_id)
{
  //
  // get data from in port & update robot model
  //
  if(m_jointDatIn.isNew ())    m_jointDatIn.read();
  if(m_basePosInitIn.isNew())  m_basePosInitIn.read();
  if(m_baseRpyInitIn.isNew())  m_baseRpyInitIn.read();
  
  if( !setInitialState() )
    return RTC::RTC_OK;


  //
  // check reference data
  //
  if( isEmpty() ) {
    return RTC::RTC_OK;
  }
  //
  // write to out port
  //
  else
    {
#ifdef LOGGING
      RTC::Time cur_time;
      setCurrentTime(cur_time);
      userLog << cur_time.sec << " " << cur_time.nsec;
      for(int i = 0; i < 3; i++)
	userLog << " " << m_jointPath->endLink()->p(i);
#endif

      // calc inverse kinematics
      hrp::Vector3  ref_p;   m_pos_interpolator->get(&ref_p[0]);
      hrp::Matrix33 ref_R0;  m_rot_interpolator->get(ref_R0);
      hrp::Matrix33 ref_R1 = m_jointPath->endLink()->calcRfromAttitude(ref_R0);

#ifdef LOGGING
      for(int i = 0; i < 3; i++)
	userLog << " " << ref_p(i);
#endif
      
      if( !m_jointPath->calcInverseKinematics(ref_p, ref_R1) ) {
	emergencyStop();
	std::cerr <<  m_instanceName << " : can't solve inverse kinematics" << std::endl;
	return RTC::RTC_OK;
      }
      
      
      // update port dat
      /*
      for(int i = 0; i < m_jointPath->numJoints(); i++) {
	int jid = m_jointPath->joint(i)->jointId;
	m_jointDat.data[jid] = m_jointPath->joint(i)->q;
      }
      */
      setReferenceState();
      setCurrentTime (m_jointDat.tm);
      setCurrentTime (m_zmpRef.tm);

      m_jointDatOut.write();
      m_zmpRefOut.write();
      m_accRefOut.write();
      m_basePosOut.write();
      m_baseRpyOut.write();


#ifdef LOGGING
      userLog << std::endl;
#endif

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


void ArmControlCartesian::setCurrentTime(RTC::Time &tm)
{
    coil::TimeValue tv(coil::gettimeofday());
    tm.sec  = tv.sec();
    tm.nsec = tv.usec()*1000;
}


bool ArmControlCartesian::setInitialState()
{
  //if(isEmpty()) return true;

  if(m_jointDat.data.length() != m_robot->numJoints()) {
    std::cerr <<  m_instanceName << " : can't determine initial posture" << std::endl;
    return false;
  }
  else {
    // set joint angles
    for (int i=0; i<m_robot->numJoints(); i++) m_robot->joint(i)->q = m_jointDat.data[i];


    // set base pos      
    hrp::Link *root = m_robot->rootLink();
    root->p = hrp::Vector3(m_basePosInit.data[0], m_basePosInit.data[1], m_basePosInit.data[2]);
    //root->p = hrp::Vector3(m_basePosInit.data[0], m_basePosInit.data[1], 0.68);


    // set base rpy
    hrp::calcRotFromRpy(root->R, m_baseRpyInit.data[0], m_baseRpyInit.data[1], m_baseRpyInit.data[2]);
    //hrp::calcRotFromRpy(root->R, 0, 0, 0);


    // update model
    m_robot->calcForwardKinematics();


    return true;
  }
}


bool ArmControlCartesian::setReferenceState()
{
  // set ref joint
  for(int i = 0; i < m_jointPath->numJoints(); i++) {
    int jid = m_jointPath->joint(i)->jointId;
    m_jointDat.data[jid] = m_jointPath->joint(i)->q;
  }


  // set ref zmp
  hrp::Link *root = m_robot->rootLink();
  hrp::Vector3 com = m_robot->calcCM();
  com[2] = 0.0;
  hrp::Vector3 local_com(tvmet::trans(root->R)*(com - root->p));

  hrp::Vector3 rpy(hrp::rpyFromRot(root->R));

  for(int i = 0; i < 3; i++) {
    m_zmpRef.data[i] = local_com[i];
    m_accRef.data[i] = 0.0;
    m_basePos.data[i]= root->p[i];
    //m_baseRpy.data[i] = rpy[0];
    m_baseRpy.data[i] = 0.0;
  }


#ifdef LOGGING
  userLog << "  basePosInit";  for(int i = 0; i < 3; i++)    userLog << " " << m_basePosInit.data[i];
  userLog << "  baseRpyInit";  for(int i = 0; i < 3; i++)    userLog << " " << m_baseRpyInit.data[i];
  userLog << "  zmpRef";       for(int i = 0; i < 3; i++)    userLog << " " << m_zmpRef.data[i];
  userLog << "  accRef";       for(int i = 0; i < 3; i++)    userLog << " " << m_accRef.data[i];
  userLog << "  basePos";      for(int i = 0; i < 3; i++)    userLog << " " << m_basePos.data[i];
  userLog << "  baseRpy";      for(int i = 0; i < 3; i++)    userLog << " " << m_baseRpy.data[i];
#endif


  return true;
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
