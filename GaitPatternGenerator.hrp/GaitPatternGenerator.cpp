// -*- c++ -*-

#include "GaitPatternGenerator.h"

#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>

#include <iostream>
#include <algorithm> // max()


//#include <fstream>
//std::ofstream user_log("/home/grxuser/Public/ogawa/src/hrp/rtc/GaitPatternGenerator/log/log.dat");


// Module specification
static const char* GaitPatternGenerator_spec[] =
  {
    "implementation_id", "GaitPatternGenerator",
    "type_name",         "GaitPatternGenerator",
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



GaitPatternGenerator::GaitPatternGenerator(RTC::Manager* manager)
  : HrprtcBaseHrp2a(manager),
	m_GaitPatternGeneratorServicePort("GaitPatternGeneratorService")
{
  m_service0.setComponent(this);
}


GaitPatternGenerator::~GaitPatternGenerator()
{
  delete m_line;
  delete m_plan;
  //delete m_prev;
}


RTC::ReturnCode_t GaitPatternGenerator::onInitialize()
{
  HrprtcBaseHrp2a::onInitialize();

  
  //
  // set port
  //
  m_GaitPatternGeneratorServicePort.registerProvider("service0", "GaitPatternGeneratorService", m_service0);
  addPort(m_GaitPatternGeneratorServicePort);

  
  //
  // get joint path
  //
  m_rleg = m_robot->getJointPath(m_robot->link("RLEG_JOINT5"), m_robot->rootLink());
  m_lleg = m_robot->getJointPath(m_robot->link("LLEG_JOINT5"), m_robot->rootLink());

  m_r2lleg = m_robot->getJointPath(m_robot->link("RLEG_JOINT5"), m_robot->link("LLEG_JOINT5"));
  m_l2rleg = m_robot->getJointPath(m_robot->link("LLEG_JOINT5"), m_robot->link("RLEG_JOINT5"));


  //
  // set initial leg position
  //
  m_robot->rootLink()->p = hrp::Vector3(0,0,0.705);
  m_robot->calcForwardKinematics();
  m_init_rleg_pos = m_robot->link("RLEG_JOINT5")->p;// 使ってない
  m_init_lleg_pos = m_robot->link("LLEG_JOINT5")->p;

  
  //
  // reference line
  //
  //m_line = new creek::QueueInterpolator(3, m_dt, creek::QUINTIC);
  m_line = new creek::QueueInterpolator(3, m_dt, creek::LINEAR);//xyt変換,時間,補間
  
  double tmp[] = {0.0, 0.0, 0.0};
  m_line->set(&tmp[0], 0.0);


  //
  // foot planner
  //
  m_plan = new creek::FootPlanner(m_dt);
  
  m_plan->setTimeParameter(0.05, 0.05, 0.05, 0.6);//歩き時間設定
  m_plan->setFootParameter(0.0, 0.095, 0.105, 0.005, 0.08); //足位置決め,zmp補正
  m_plan->calcFootParameter(hrp::Vector3(0.0), m_robot->link("RLEG_JOINT5")->p, m_robot->link("LLEG_JOINT5")->p);//使ってない
  
  
  //
  // preview control
  //
  m_prev = new creek::PreviewControl(m_dt, 0.814);//予見制御

  RTC::Properties& prop = getProperties();
  std::string fname_kgain, fname_fgain;
  fname_kgain = prop["preview_gain.k"];
  fname_fgain = prop["preview_gain.f"];

  if( !m_prev->loadGain(fname_kgain, fname_fgain) ) {
	std::cerr << m_instanceName << " : failed to load preview gain" << std::endl;
  }
  else {
	std::cout << m_instanceName << " : load preview gain k\n  file = " << fname_kgain << std::endl;
	std::cout << m_instanceName << " : load preview gain f\n  file = " << fname_fgain << std::endl;
  }

 
  return RTC::RTC_OK;
}


RTC::ReturnCode_t GaitPatternGenerator::onExecute(RTC::UniqueId ec_id)
{
  if( m_jointDatIn.isNew() )    m_jointDatIn.read();
  if( m_basePosInitIn.isNew() ) m_basePosInitIn.read();
  if( m_baseRpyInitIn.isNew() ) m_baseRpyInitIn.read();

  
  if( !m_plan->isInit()  ||  !m_prev->isInit() )
	return RTC::RTC_OK;


  //updateModel();

  
  if( !m_line->empty() ) {
	double tmp[3];
	m_line->get(&tmp[0]);
	m_plan->set(tmp[0], tmp[1], tmp[2]);
  }


  if( m_plan->zmpSequence()->size() <= m_prev->numPreview() && !m_plan->endIsStop() ) {
	std::cout << m_instanceName << " : add stop motion" << std::endl;
	m_plan->stop();
  }
  
  
  if( m_plan->zmpSequence()->size() > m_prev->numPreview() ) {


	// cur support foot
	creek::FootType sup = m_plan->supportFoot();
	

	// set ref data
	hrp::Vector3 ref_com_pos;
	hrp::Vector3 ref_com_vel;
	hrp::Vector3 ref_com_acc;
	m_prev->get(m_plan->zmpSequence(), ref_com_pos, ref_com_vel, ref_com_acc);

	
	//hrp::Matrix33 ref_rot(tvmet::identity<hrp::Matrix33>());
	hrp::Vector3 ref_rfoot_pos;  hrp::Matrix33 ref_rfoot_rot;
	hrp::Vector3 ref_lfoot_pos;  hrp::Matrix33 ref_lfoot_rot;
	hrp::Vector3 ref_zmp;        hrp::Matrix33 ref_waist_rot;

	m_plan->get(ref_zmp, ref_rfoot_pos, ref_rfoot_rot, ref_lfoot_pos, ref_lfoot_rot, ref_waist_rot);
	

	/*
	for(int i = 0; i < 3; i++) user_log << "  " << ref_com_pos(i);
	for(int i = 0; i < 3; i++) user_log << "  " << ref_zmp(i);
	for(int i = 0; i < 3; i++) user_log << "  " << ref_rfoot_pos(i);
	for(int i = 0; i < 3; i++) user_log << "  " << ref_lfoot_pos(i);
	user_log << std::endl;
	*/


	// calc com inv
	if( sup < 0 ) {
	  if( !calcComInverseKinematics(ref_com_pos, m_r2lleg, ref_lfoot_pos, ref_lfoot_rot, ref_waist_rot) ) {
		std::cerr << m_instanceName << " : cannot calc com inverse (support foot = right)" << std::endl;
		return RTC::RTC_OK;
	  }
	}
	else {
	  if( !calcComInverseKinematics(ref_com_pos, m_l2rleg, ref_rfoot_pos, ref_rfoot_rot, ref_waist_rot) ) {
		std::cerr << m_instanceName << " : cannot calc com inverse (support foot = left)" << std::endl;
		return RTC::RTC_OK;
	  }
	}
	
	hrp::Vector3 ref_zmp_waist;
	ref_zmp_waist = tvmet::trans(m_robot->rootLink()->R) * (ref_zmp - m_robot->rootLink()->p);


	
	// update outport
	/*
	for(int i = 0; i < m_r2lleg->numJoints(); i++) {
	  int jid = m_r2lleg->joint(i)->jointId;
	  m_jointDat.data[jid] = m_r2lleg->joint(i)->q;
	  }
	*/
	for(int i = 0; i < m_robot->numJoints(); i++)
	  m_jointDat.data[i] = m_robot->joint(i)->q;
	
	for(int i = 0; i < 3; i++) {
	  m_zmpRef.data[i]  = ref_zmp_waist(i);
	  m_accRef.data[i]  = ref_com_acc(i);
	  m_basePos.data[i] = m_robot->rootLink()->p(i);
	  m_baseRpy.data[i] = 0.0;
	}

	
	m_jointDatOut.write();
	m_zmpRefOut.write();
	m_accRefOut.write();
	m_basePosOut.write();
	m_baseRpyOut.write();
  }
  
  
  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------

void GaitPatternGenerator::updateModel()
{
  for(int i = 0; i < m_robot->numJoints(); i++)
	m_robot->joint(i)->q = m_jointDat.data[i];

  
  if( m_plan->supportFoot() < 0 ) {
	//m_r2lleg->baseLink()->p = m_init_rleg_pos;
	//m_r2lleg->baseLink()->R = tvmet::identity<hrp::Matrix33>();
	m_r2lleg->calcForwardKinematics();
  }
  else {
	//m_l2rleg->baseLink()->p = m_init_lleg_pos;
	//m_l2rleg->baseLink()->R = tvmet::identity<hrp::Matrix33>();
	m_l2rleg->calcForwardKinematics();
  }

  
  m_robot->calcForwardKinematics();
}


void GaitPatternGenerator::setTargetPos(double x, double y, double th, double time)
{
  if( m_plan->empty() )
	updateModel();


  double pre_end[3];
  if( m_line->empty() ) {	
	m_line->get(&pre_end[0], false);
  }
  else {
	for(int i = 0; i < 3; i++)
	  pre_end[i] = m_line->back()[i];
  }
  double dx, dy, dth;
  dx  =  x - pre_end[0];
  dy  =  y - pre_end[1];
  dth = th - pre_end[2];

    
  double ref_time = std::max( 8.5*sqrt(dx*dx+dy*dy), 4.0*sqrt(dth*dth));
  ref_time = std::max(ref_time, time);
  ref_time = std::max(ref_time, 3.0);
  //std::cout << m_instanceName << " : input time = " << time << "    |  motion time = " << ref_time << std::endl;
  printf("%s : input time = %6.2f    |  motion time = %6.2f\n", m_instanceName.c_str(), time, ref_time);

  
  double tmp[] = {x,y,th};
  m_line->set(&tmp[0], ref_time);
  

  if( !m_prev->isInit() ) {
	hrp::Vector3 cur_com(m_robot->calcCM());
	m_prev->init(cur_com);
  }

  
  if( !m_plan->isInit() ) {
	hrp::Vector3 cur_com(m_robot->calcCM());
	hrp::Vector3 origin;  m_line->get(&origin[0], false);
	hrp::Vector3 zmp(cur_com);  zmp(2) = 0.0;
	m_plan->init(m_robot->link("RLEG_JOINT5")->p, m_robot->link("RLEG_JOINT5")->R, m_robot->link("LLEG_JOINT5")->p, m_robot->link("LLEG_JOINT5")->R, zmp, 3.0, creek::LFOOT);
  }
}


void GaitPatternGenerator::stop()
{
  m_plan->stop();
}


bool GaitPatternGenerator::calcComInverseKinematics(const hrp::Vector3 &ref_com, const hrp::JointPathPtr legs, const hrp::Vector3 &ref_swing_foot_pos, const hrp::Matrix33 &ref_swing_foot_rot, const hrp::Matrix33 ref_waist_rot)
{
  //
  // set variables
  //
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  hrp::JointPathPtr support_leg = m_robot->getJointPath(legs->baseLink(), m_robot->rootLink());
  //const hrp::Matrix33 ref_waist_rot(tvmet::identity<hrp::Matrix33>());

  int n = legs->numJoints();
  std::vector<double> qorg(n);
  for(int i=0; i < n; ++i){
    qorg[i] = legs->joint(i)->q;
  }



  //
  // set val conf
  //
  hrp::dmatrix J_com(3, m_robot->numJoints());
  hrp::dmatrix J_sup(6,support_leg->numJoints());
  hrp::dmatrix J_leg(6,n);
  hrp::dmatrix J(12,n);

  hrp::dvector v(12);
  hrp::dvector dq(n);

  double maxIKErrorSqr = 1.0e-6 * 1.0e-6 * 1.0e-6 * 1.0e-6;
  double errsqr = maxIKErrorSqr * 100.0;
  bool converged = false;



  //
  // main loop
  //
  for(int k=0; k < MAX_IK_ITERATION; k++){

    //
    // calc velocity
    //
    hrp::Vector3 cur_com(m_robot->calcCM());
    hrp::Vector3 dp_com(ref_com-cur_com);
    hrp::Vector3 omega_waist(m_robot->rootLink()->R * hrp::omegaFromRot(hrp::Matrix33(tvmet::trans(m_robot->rootLink()->R) * ref_waist_rot)));
    hrp::Vector3 dp_swg(ref_swing_foot_pos - legs->endLink()->p);
    hrp::Vector3 omega_swg(legs->endLink()->R * hrp::omegaFromRot(hrp::Matrix33(tvmet::trans(legs->endLink()->R) * ref_swing_foot_rot)));
  
    hrp::setVector3(dp_com      , v, 0);
    hrp::setVector3(omega_waist , v, 3);
    hrp::setVector3(dp_swg      , v, 6);
    hrp::setVector3(omega_swg   , v, 9);


    errsqr = hrp::ublas::inner_prod(v,v);
    if( errsqr < maxIKErrorSqr ) {
      converged = true;
      break;
    }
    else if( MAX_IK_ITERATION == 1) {
      converged = true;
    }
   


    //
    // calc jacobians
    //
    // calc com jacobian
    hrp::Link*   base = legs->baseLink();
    m_robot->calcCMJacobian(base, J_com);

    // calc sup jacobian
    support_leg->calcJacobian(J_sup);

    // calc legs jacobian
    legs->calcJacobian(J_leg);

    // calc all Jacobian
    J = hrp::dzeromatrix(12,12);


    // set com jacobian
    for(int i = 0; i < legs->numJoints(); i++) {
      int id = legs->joint(i)->jointId;
      for(int j = 0; j < 3; j++) {
		J(j,i) = J_com(j,id);
      }
    }

    // set sup jacobian
    for(int i = 0; i < support_leg->numJoints(); i++) {
      for(int j = 3; j < 6; j++) {
		J(j,i) = J_sup(j,i);
      } 
    }

    // set legs jacobian
    for(int i = 0; i < legs->numJoints(); i++) {
      for(int j = 0; j < 6; j++) {
		J(j+6,i) = J_leg(j,i);
      } 
    }

	
	/*
	if( k==0 )
	  user_log << J << "\n\n" << J_com << "\n\n\n" << std::endl;
	*/
	

    //
    // solve quation
    //
    hrp::solveLinearEquationLU(J, v, dq);
  
    for(int i = 0; i < n; i++) {
      legs->joint(i)->q += LAMBDA * dq(i);
    }
    legs->calcForwardKinematics();
    m_robot->calcForwardKinematics();

  }



  if(!converged){
    for(int i=0; i < n; ++i){
      legs->joint(i)->q = qorg[i];
    }
    legs->calcForwardKinematics();
    m_robot->calcForwardKinematics();
  }



  return converged;
}



//-----------------------------------------------------------------------

extern "C"
{
 
  void GaitPatternGeneratorInit(RTC::Manager* manager)
  {
    coil::Properties profile(GaitPatternGenerator_spec);
    manager->registerFactory(profile,
                             RTC::Create<GaitPatternGenerator>,
                             RTC::Delete<GaitPatternGenerator>);
  }
  
};
