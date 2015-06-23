// -*- C++ -*-

#include "creekSequencePlayer.h"

#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>

static const char* creeksequenceplayer_spec[] =
  {
    "implementation_id", "creekSequencePlayer",
    "type_name",         "creekSequencePlayer",
    "description",       "SequencePlayer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequencePlayer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekSequencePlayer::creekSequencePlayer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qInitIn("qInit", m_qInit),
    m_qRefOut("qRef", m_qRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_creekSequencePlayerServicePort("creekSequencePlayerService"),
    m_waitFlag(false),
    m_waitSem(0)
{
  m_service0.setComponent(this);
}

creekSequencePlayer::~creekSequencePlayer()
{
}


RTC::ReturnCode_t creekSequencePlayer::onInitialize()
{
  // Set InPort buffers
  addInPort("qInit", m_qInitIn);

  // Set OutPort buffer
  addOutPort("qRef", m_qRefOut);
  addOutPort("basePos", m_basePosOut);
  addOutPort("baseRpy", m_baseRpyOut);
  addOutPort("zmpRef", m_zmpRefOut);

  // Set service provider to Ports
  m_creekSequencePlayerServicePort.registerProvider("service0", "creekSequencePlayerService", m_service0);

  // Set CORBA Service Ports
  addPort(m_creekSequencePlayerServicePort);


  //
  // get properties
  //
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());


  //
  // setup robot model
  //
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str() );
  m_dof = m_robot->numJoints();


  //
  // init interpolator
  //
  m_seq[ANGLES] = new creek::QueueInterpolator(m_dof, m_dt, creek::CUBIC);
  for(int i=POS; i<NUM_SEQ; i++)
    m_seq[i] = new creek::QueueInterpolator(3, m_dt, creek::CUBIC);


  //
  // init data port
  //
  m_qRef.data.length(m_dof);
  m_qInit.data.length(m_dof);
  for(unsigned int i=0; i<m_dof; i++) m_qInit.data[i] = 0.0;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onActivated(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() ) m_qInitIn.read();


  //
  // init model
  //
  RTC::Properties& prop = getProperties();

  std::vector<double> tmp;
  coil::stringTo(tmp, prop["initBasePos"].c_str());
  m_robot->rootLink()->p() << tmp[0], tmp[1], tmp[2];

  coil::stringTo(tmp, prop["initBaseRpy"].c_str());
  m_robot->rootLink()->R() = cnoid::rotFromRpy(tmp[3], tmp[4], tmp[5]);
  
  for(unsigned int i=0; i<m_dof; i++) m_robot->joint(i)->q() = m_qInit.data[i];
  m_robot->calcForwardKinematics();


  //
  // init interpolator
  //
  for(int i=0; i<NUM_SEQ; i++) m_seq[i]->clear();

  m_seq[POS]->set(m_robot->rootLink()->p().data());
  m_seq[RPY]->set(&tmp[0]);

  cnoid::Vector3 zmp(m_robot->calcCenterOfMass());
  zmp(2) = 0.0;
  cnoid::Vector3 zmpWaist( m_robot->rootLink()->R().transpose() * (zmp-m_robot->rootLink()->p()) );
  m_seq[ZMP]->set(zmpWaist.data());



  if( isEmpty() && m_waitFlag ) {
    m_waitFlag = false;
    m_waitSem.post();
  }


  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onExecute(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() ) m_qInitIn.read();


  if( isEmpty() && m_waitFlag ) {
    m_waitFlag = false;
    m_waitSem.post();
  }
 

  if( !m_seq[ANGLES]->empty() ) {
    m_seq[ANGLES]->get(m_qRef.data.get_buffer());
    m_qRefOut.write();
  }


  double tmp[3];
  if( !m_seq[POS]->empty() )  {
    m_seq[POS]->get(&tmp[0]);
    m_basePos.data.x = tmp[0];
    m_basePos.data.y = tmp[1];
    m_basePos.data.z = tmp[2];
    m_basePosOut.write();
  }

  if( !m_seq[RPY]->empty() )  {
    m_seq[RPY]->get(&tmp[0]);
    m_baseRpy.data.r = tmp[0];
    m_baseRpy.data.p = tmp[1];
    m_baseRpy.data.y = tmp[2];
    m_baseRpyOut.write();
  }

  if( !m_seq[ZMP]->empty() )  {
    m_seq[ZMP]->get(&tmp[0]);
    m_zmpRef.data.x = tmp[0];
    m_zmpRef.data.y = tmp[1];
    m_zmpRef.data.z = tmp[2];
    m_zmpRefOut.write();
  }



  return RTC::RTC_OK;
}


//----------------------------------------------------------------------------------


void creekSequencePlayer::waitInterpolation()
{
  m_waitFlag = true;
  m_waitSem.wait();
}


bool creekSequencePlayer::setJointAngles(const double *angles, double tm)
{
  if( m_qInit.data.length() != m_dof ) return false;

  if( m_seq[ANGLES]->empty() ) {
    m_seq[ANGLES]->clear();
    m_seq[ANGLES]->set(m_qInit.data.get_buffer());
    return m_seq[ANGLES]->set(angles, tm);
  }
  else
    return false;
}


bool creekSequencePlayer::setJointAngle(const char* jname, double jv, double tm)
{
  if( m_qInit.data.length() != m_dof ) return false;
  
  if( m_seq[ANGLES]->empty() ) {
    cnoid::Link *link = m_robot->link(jname);
    if( link ) {
      double angles[m_dof];
      std::memcpy(&angles[0], m_qInit.data.get_buffer(), m_dof*sizeof(double));
      angles[link->jointId()] = jv;

      m_seq[ANGLES]->clear();
      m_seq[ANGLES]->set(m_qInit.data.get_buffer());
      return m_seq[ANGLES]->set(angles, tm);
    }
  }
  else
    return false;
}


bool creekSequencePlayer::setBasePos(const double *pos, double tm)
{
  if( !m_seq[POS]->empty() ) 
    return false;
  else
    return m_seq[POS]->set(pos, tm);
}


bool creekSequencePlayer::setBaseRpy(const double *rpy, double tm)
{
  if( !m_seq[RPY]->empty() ) 
    return false;
  else
    return m_seq[RPY]->set(rpy, tm);
}


bool creekSequencePlayer::setZmp(const double *zmp, double tm)
{
  if( !m_seq[ZMP]->empty() ) 
    return false;
  else
    return m_seq[ZMP]->set(zmp, tm);
}


bool creekSequencePlayer::isEmpty()
{
  for(int i=0; i<NUM_SEQ; i++)
    if( !m_seq[i]->empty() ) return false;

  return true;
}


void creekSequencePlayer::calcCoM()
{
  for(unsigned int i=0; i<m_dof; i++)
    m_robot->joint(i)->q() = m_qInit.data[i];
  m_robot->calcForwardKinematics();
  
  cnoid::Vector3 ground((m_robot->joint(5)->p() + m_robot->joint(11)->p()) / 2.0);
  cnoid::Vector3 com(m_robot->calcCenterOfMass());

  cnoid::Vector3 com_pos(com-ground);
  
  std::cout << "creekSequencePlayer\n"
	    << "com pos = " << com_pos(0) << ", " << com_pos(1) << ", " << com_pos(2)
	    << "\n"
	    << "ground  = " << ground(0)  << ", " << ground(1)  << ", " << ground(2)
	    << std::endl << std::endl;
}


//----------------------------------------------------------------------------------


extern "C"
{
 
  void creekSequencePlayerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creeksequenceplayer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekSequencePlayer>,
                             RTC::Delete<creekSequencePlayer>);
  }
  
};



