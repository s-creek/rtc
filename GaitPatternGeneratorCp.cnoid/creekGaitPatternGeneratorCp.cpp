// -*- C++ -*-

#include "creekGaitPatternGeneratorCp.h"
//#include "../util/CheckCounter.h"

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

#include <creekGeometry.h>
#include <fstream>
std::ofstream logger("/home/ogawa/workspace/git/rtc/GaitPatternGeneratorCp.cnoid/log/log.dat");

static const char* creekgaitpatterngeneratorcp_spec[] =
  {
    "implementation_id", "creekGaitPatternGeneratorCp",
    "type_name",         "creekGaitPatternGeneratorCp",
    "description",       "creekGaitPatternGeneratorCp",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekGaitPatternGeneratorCp",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekGaitPatternGeneratorCp::creekGaitPatternGeneratorCp(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qInitIn("qInit", m_qInit),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
    m_qRefOut("qRef", m_qRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_contactStatesOut("contactStates", m_contactStates),
    m_creekGaitPatternGeneratorCpServicePort("creekGaitPatternGeneratorCpService")
{
  m_service0.setComponent(this);
}


creekGaitPatternGeneratorCp::~creekGaitPatternGeneratorCp()
{
}


RTC::ReturnCode_t creekGaitPatternGeneratorCp::onInitialize()
{
  std::cout << "creekGaitPatternGeneratorCp : onInitialize" << std::endl;


  // Set InPort buffers
  addInPort("qInit", m_qInitIn);
  addInPort("basePosInit", m_basePosInitIn);
  addInPort("baseRpyInit", m_baseRpyInitIn);
  addInPort("zmpRefInit", m_zmpRefInitIn);

  // Set OutPort buffer
  addOutPort("qRef", m_qRefOut);
  addOutPort("basePos", m_basePosOut);
  addOutPort("baseRpy", m_baseRpyOut);
  addOutPort("zmpRef", m_zmpRefOut);
  addOutPort("contactStates", m_contactStatesOut);

  // Set service provider to Ports
  // Set CORBA Service Ports
  m_creekGaitPatternGeneratorCpServicePort.registerProvider("service0", "creekGaitPatternGeneratorCpService", m_service0);
  addPort(m_creekGaitPatternGeneratorCpServicePort);


  // get properties
  RTC::Properties& prop = getProperties();
  double dt;
  coil::stringTo(dt, prop["dt"].c_str());


  // set up robot
  m_robot = new Robot;
  cnoid::BodyLoader bl;
  if( !bl.load( m_robot, prop["model"] ) ) {
    std::cerr << "creekGaitPatternGeneratorCp : model load error" << std::endl;
  }
  if( !m_robot->setJointPath( prop["RLEG_END"], prop["BASE_LINK"], prop["LLEG_END"], prop["BASE_LINK"]) ){
    std::cerr << "creekGaitPatternGeneratorCp : set joint path error" << std::endl;
  }
  m_robot->rootLink()->p() << 0.0, 0.0, 0.846;
  m_robot->setSoleOffset(creek::Vector3(0.0, 0.0, -0.10), creek::Vector3(0.0, 0.0, -0.10));

  
  // set up planner
  m_planner.setRobot(m_robot);
  m_planner.setTime(dt, 0.7, 0.1);
  m_planner.setFootSize(0.13, 0.10, 0.075, 0.055);
  m_planner.setOffset(0.015, -0.015);
  //m_planner.setOffset(0.015, -0.01);
  m_planner.setMargin(0.01);


  // init data port
  m_qRef.data.length(m_robot->numJoints());
  m_contactStates.data.length(2);
  m_contactStates.data[0]=m_contactStates.data[1]=1;


  //SET_CHECK_COUNTER;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitPatternGeneratorCp::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekGaitPatternGeneratorCp : onActivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitPatternGeneratorCp::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekGaitPatternGeneratorCp : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitPatternGeneratorCp::onExecute(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() )       m_qInitIn.read();
  if( m_basePosInitIn.isNew() ) m_basePosInitIn.read();
  if( m_baseRpyInitIn.isNew() ) m_baseRpyInitIn.read();
  if( m_zmpRefInitIn.isNew() )  m_zmpRefInitIn.read();


  //CHECK_COUNTER(cc::m_stepCounter);


  if( m_planner.step().empty() )
    return RTC::RTC_OK;


  static int minNum(10);
  //
  // stepping mode
  //
  if( m_planner.isStepping() ) {
    if( m_planner.step().size() < minNum ) {
      //boost::mutex::scoped_lock lock(mtx);
      creek::FootType sup = m_planner.getFootType( m_planner.step().back().zmp );
      if( sup == creek::RFOOT ) {
	m_planner.addStep(m_rfootRef, creek::RFOOT);
      }
      else {
	m_planner.addStep(m_lfootRef, creek::LFOOT);
      }
      //lock.unlock();
    }
  }


  //
  // add step or stop
  //
  if( m_planner.step().size() < minNum ) {
    //boost::mutex::scoped_lock lock(mtx);
    creek::FootType swg = m_planner.getSwingFootType( m_rfootRef, m_lfootRef );
    switch(swg)
      {
      case creek::RFOOT:
	m_planner.addStep(m_rfootRef, creek::RFOOT);
	break;
      case creek::LFOOT:
	m_planner.addStep(m_lfootRef, creek::LFOOT);
	break;
      case creek::DFOOT:
      default:
	m_planner.addStop();
      }
    //lock.unlock();
  }


  //
  // update model
  //
  //boost::mutex::scoped_lock lock(mtx);
  creek::StepData step;
  m_planner.get(step);
  //lock.unlock();

  logger << " " << step.com[0] << " " << step.com[1];
  logger << " " << step.zmp[0] << " " << step.zmp[1];
  logger << " " << step.cp[0]  << " " << step.cp[1];
  logger << " " << step.zmp[2];
  logger << std::endl;

  m_waistRef = creek::midYaw(step.rfoot.linear(), step.lfoot.linear());
  if( !m_robot->calc(step.sup, step.com, m_waistRef,
		     step.rfoot.translation(), step.rfoot.linear(),
		     step.lfoot.translation(), step.lfoot.linear()) ) {
    std::cout << "creekGaitPatternGeneratorCp : com inverse error" << std::endl;
  }
  else {
    for(int i=0; i<m_robot->numJoints(); i++) {
      m_qRef.data[i] = m_robot->joint(i)->q();
    }
    m_basePos.data.x = m_robot->rootLink()->p()[0];
    m_basePos.data.y = m_robot->rootLink()->p()[1];
    m_basePos.data.z = m_robot->rootLink()->p()[2];

    cnoid::Vector3 rpy = m_robot->rootLink()->R().eulerAngles(2,1,0);
    m_baseRpy.data.r = rpy[2];
    m_baseRpy.data.p = rpy[1];
    m_baseRpy.data.y = rpy[0];

    cnoid::Vector3 zmpRel = m_robot->rootLink()->R().transpose() * (step.zmp - m_robot->rootLink()->p());
    m_zmpRef.data.x = zmpRel[0];
    m_zmpRef.data.y = zmpRel[1];
    m_zmpRef.data.z = zmpRel[2];

    switch( step.sup )
      {
      case creek::RFOOT:
	m_contactStates.data[0] = 1;
	m_contactStates.data[1] = 0;
	break;
      case creek::LFOOT:
	m_contactStates.data[0] = 0;
	m_contactStates.data[1] = 1;
	break;
      case creek::DFOOT:
	m_contactStates.data[0] = 1;
	m_contactStates.data[1] = 1;
	break;
      default:
	m_contactStates.data[0] = 0;
	m_contactStates.data[1] = 0;
      }


    m_qRefOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpRefOut.write();
    m_contactStatesOut.write();
  }

  
  return RTC::RTC_OK;
}


//----------------------------------------------------------------------------------


void creekGaitPatternGeneratorCp::startStepping()
{
  initPlanner();  
  m_planner.setStepping(true);
}


void creekGaitPatternGeneratorCp::stopStepping()
{
  m_planner.setStepping(false);
}


void creekGaitPatternGeneratorCp::test()
{
  //boost::mutex::scoped_lock lock(mtx);
  initPlanner();
 
  int testMode = 6;
  if( testMode == 0 ) {
    m_planner.setStepping(false);
    m_planner.setTime(0.005, 1.3, 0.2);
 
    m_rfootRef.translation()[2] += 0.1;
    m_rfootRef.linear() = Eigen::AngleAxisd(-0.4, creek::Vector3::UnitY()).toRotationMatrix();
    m_planner.addStep(m_rfootRef, creek::RFOOT, 1);
 
    m_planner.step().back().sup = creek::DFOOT;
    creek::Vector3 lp = m_planner.step().back().lfoot.translation();
    creek::Matrix3 lR = m_planner.step().back().lfoot.linear();
    creek::Vector3 cpRef = lp + lR * creek::Vector3(0.01, 0.0, 0.0);
    m_planner.addComSequence(cpRef, 10.0, creek::LFOOT); 
  }
  else if( testMode == 1 ){
    m_rfootRef.translation()[0] += 0.1;
    m_planner.addStep(m_rfootRef, creek::RFOOT);

    m_lfootRef.translation()[0] += 0.2;
    m_planner.addStep(m_lfootRef, creek::LFOOT);

    m_rfootRef.translation()[0] += 0.2;
    m_planner.addStep(m_rfootRef, creek::RFOOT);

    m_lfootRef.translation()[0] += 0.15;
    m_lfootRef.translation()[1] += 0.05;
    m_lfootRef.linear() = Eigen::AngleAxisd(0.2, creek::Vector3::UnitZ()) * m_lfootRef.linear();
    m_planner.addStep(m_lfootRef, creek::LFOOT);

    m_planner.addStop(true);
  }
  else if( testMode == 2 ){
    m_planner.setTime(0.005, 1.0, 0.2);
    //m_planner.setOffset(0.03, -0.015);

    m_rfootRef.translation()[0] += 0.32;
    m_rfootRef.translation()[2] += 0.05;
    m_planner.addStep(m_rfootRef, creek::RFOOT);
    
    m_lfootRef.translation()[0] += 0.32;
    m_lfootRef.translation()[2] += 0.05;
    m_planner.addStep(m_lfootRef, creek::LFOOT);
    /*
    m_rfootRef.translation()[0] += 0.2;
    m_planner.addStep(m_rfootRef, creek::RFOOT);
    
    m_lfootRef.translation()[0] += 0.2;
    m_planner.addStep(m_lfootRef, creek::LFOOT);
    */
    m_planner.addStop(true);
  }
  else if( testMode == 3 ) {
    creek::FootType sup = m_planner.getFootType( m_planner.step().back().zmp );

    if( sup == creek::RFOOT ) {
      m_rfootRef.translation()[0] += 0.15;
      m_planner.addStep(m_rfootRef, creek::RFOOT);

      m_lfootRef.translation()[0] += 0.3;
      m_planner.addStep(m_lfootRef, creek::LFOOT);
    }
    else {
      m_lfootRef.translation()[0] += 0.15;
      m_planner.addStep(m_lfootRef, creek::LFOOT);
    }

    m_rfootRef.translation()[0] += 0.3;
    m_planner.addStep(m_rfootRef, creek::RFOOT);

    m_lfootRef.translation()[0] += 0.3;
    m_planner.addStep(m_lfootRef, creek::LFOOT);

    m_rfootRef.translation()[0] += 0.3;
    m_planner.addStep(m_rfootRef, creek::RFOOT);
    
    m_lfootRef.translation()[0] += 0.15;
    m_planner.addStep(m_lfootRef, creek::LFOOT);
  }
  else if( testMode == 4 ) {
    creek::FootType swg = m_planner.getFootType( m_planner.step().back().zmp );

    for(int i=0; i<5; i++) {
      m_rfootRef.translation()[0] += 0.1;
      m_lfootRef.translation()[0] += 0.1;
      
      if( swg == creek::RFOOT ) {
	m_planner.addStep(m_rfootRef, creek::RFOOT);
	swg = creek::LFOOT;
      }
      else {
	m_planner.addStep(m_lfootRef, creek::LFOOT);
	swg = creek::RFOOT;
      }
    }
    if( swg == creek::RFOOT )
      m_planner.addStep(m_rfootRef, creek::RFOOT);
    else
      m_planner.addStep(m_lfootRef, creek::LFOOT);
  }
  else if( testMode == 5 ) {
    creek::Matrix3 waistR  = m_robot->rootLink()->R();
    creek::Vector3 waist2r = waistR.transpose() *(m_robot->rfoot()->p() - m_robot->rootLink()->p());
    creek::Vector3 waist2l = waistR.transpose() *(m_robot->lfoot()->p() - m_robot->rootLink()->p());

    creek::FootType swg = m_planner.getFootType( m_planner.step().back().zmp );

    for(int i=0; i<2; i++) {
      waistR = Eigen::AngleAxisd(15.0/180.0*M_PI, creek::Vector3::UnitZ()) * waistR;

      m_rfootRef.translation() = m_robot->rootLink()->p() + waistR * waist2r;
      m_lfootRef.translation() = m_robot->rootLink()->p() + waistR * waist2l;
      
      m_rfootRef.linear() = waistR;
      m_lfootRef.linear() = waistR;
      
      if( swg == creek::RFOOT ) {
	m_planner.addStep(m_rfootRef, creek::RFOOT);
	swg = creek::LFOOT;
      }
      else {
	m_planner.addStep(m_lfootRef, creek::LFOOT);
	swg = creek::RFOOT;
      }
    }
    std::cout << "test " << waistR << std::endl;
    if( swg == creek::RFOOT )
      m_planner.addStep(m_rfootRef, creek::RFOOT);
    else
      m_planner.addStep(m_lfootRef, creek::LFOOT);
  }
  else if( testMode == 6 ) {
    creek::Matrix3 orgR;
    creek::Vector3 orgP, toR, toL;

    orgP = m_robot->rootLink()->p() + m_robot->rootLink()->R() * creek::Vector3(0, 1.0, 0);
    orgR = m_robot->rootLink()->R();    
    toR = orgR.transpose() * (m_robot->rfoot()->p() - orgP);
    toL = orgR.transpose() * (m_robot->lfoot()->p() - orgP);

    std::cout << "org = " << orgP.format(creek::IOvec()) << std::endl;

    creek::FootType swg = m_planner.getFootType( m_planner.step().back().zmp );
    for(int i=0; i<18; i++) {
      orgR = Eigen::AngleAxisd(5.0/180.0*M_PI, creek::Vector3::UnitZ()) * orgR;

      m_rfootRef.translation() = orgP + orgR * toR;
      m_lfootRef.translation() = orgP + orgR * toL;
      
      m_rfootRef.linear() = orgR;
      m_lfootRef.linear() = orgR;

      if( swg == creek::RFOOT ) {
	m_planner.addStep(m_rfootRef, creek::RFOOT);
	swg = creek::LFOOT;
      }
      else {
	m_planner.addStep(m_lfootRef, creek::LFOOT);
	swg = creek::RFOOT;
      }
    }
    if( swg == creek::RFOOT )
      m_planner.addStep(m_rfootRef, creek::RFOOT);
    else
      m_planner.addStep(m_lfootRef, creek::LFOOT);
  }
  //lock.unlock();
}


void creekGaitPatternGeneratorCp::initPlanner()
{
  if( !m_planner.step().empty() )
    return;


  for(int i=0; i<m_robot->numJoints(); i++) {
    m_robot->joint(i)->q() = m_qInit.data[i];
  }
  m_robot->rootLink()->p() << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
  m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y);
  m_robot->calcForwardKinematics();


  if( m_zmpRefInitIn.isEmpty() || true ) {
    cnoid::Vector3 com = m_robot->calcCenterOfMass();
    cnoid::Vector3 ground = (m_robot->rfoot()->p() + m_robot->lfoot()->p()) / 2.0;
    cnoid::Vector3 zmp(com);
    zmp[2] = ground[2];
    m_planner.initParameters(zmp);
  }
  else {
    cnoid::Vector3 zmpRel;
    zmpRel << m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z;
    cnoid::Vector3 zmp = m_robot->rootLink()->p() + m_robot->rootLink()->R() * zmpRel;
    m_planner.initParameters(zmp);
  }

  
  m_waistRef = m_robot->rootLink()->R();
  m_rfootRef = m_robot->rfoot()->position();
  m_lfootRef = m_robot->lfoot()->position();
}


//----------------------------------------------------------------------------------


extern "C"
{
 
  void creekGaitPatternGeneratorCpInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekgaitpatterngeneratorcp_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekGaitPatternGeneratorCp>,
                             RTC::Delete<creekGaitPatternGeneratorCp>);
  }
  
};



