// -*- C++ -*-

#include "creekSequencePlayer.h"

#include <cnoid/BodyLoader>
#include <cnoid/Link>

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
    m_creekSequencePlayerServicePort("creekSequencePlayerService")
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


  m_qRef.data.length(m_dof);


  //
  // init interpolator
  //
  m_seqAngles  = new creek::Interpolator(m_dof, m_dt, creek::CUBIC);
  m_seqBasePos = new creek::Interpolator(3, m_dt, creek::CUBIC);
  m_seqBaseRpy = new creek::Interpolator(3, m_dt, creek::CUBIC);
  m_seqZmp     = new creek::Interpolator(3, m_dt, creek::CUBIC);


  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onActivated(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() ) m_qInitIn.read();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onExecute(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() ) m_qInitIn.read();

  if( !m_seqAngles->empty() ) {
    m_seqAngles->get(m_qRef.data.get_buffer());
    m_qRefOut.write();
  }

  return RTC::RTC_OK;
}


//----------------------------------------------------------------------------------


bool creekSequencePlayer::setJointAngles(const double *angles, double tm)
{
  std::cout << "setJointAngles" << std::endl;
  if( m_qInit.data.length() != m_dof ) return false;

  calcCoM();

  return m_seqAngles->calcInterpolation(m_qInit.data.get_buffer(), angles, tm);
}


bool creekSequencePlayer::isEmpty()
{
  calcCoM();
  return m_seqAngles->empty();
}


void creekSequencePlayer::calcCoM()
{
  for(unsigned int i=0; i<m_dof; i++)
    m_robot->joint(i)->q() = m_qInit.data[i];
  m_robot->calcForwardKinematics();
  
  cnoid::Vector3 ground((m_robot->joint(5)->p() + m_robot->joint(11)->p()) / 2.0);
  cnoid::Vector3 com(m_robot->calcCenterOfMass());

  cnoid::Vector3 com_pos(com-ground);
  
  std::cout << "com pos = " << com_pos(0) << ", " << com_pos(1) << ", " << com_pos(2)
	    << "\n"
	    << "ground  = " << ground(0)  << ", " << ground(1)  << ", " << ground(2)
	    << std::endl;
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



