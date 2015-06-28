#include "creekPdServo.h"
#include "VectorConvert.h"

#include <rtm/CorbaNaming.h>

// Module specification
static const char* module_spec[] =
  {
    "implementation_id", "creekPdServo",
    "type_name",         "creekPdServo",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekPdServo::creekPdServo(RTC::Manager * manager)
  : RTC::DataFlowComponentBase(manager),
    m_qRefIn("qRef", m_qRef),
    m_qCurIn("q", m_qCur),
    m_tauRefOut("tau", m_tauRef)
{
}


RTC::ReturnCode_t creekPdServo::onInitialize()
{
  //
  // Registration: InPort/OutPort/Service
  //
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("q", m_qCurIn);
  
  // Set OutPort buffer
  addOutPort("tau", m_tauRefOut);


  //
  // get properties
  //
  RTC::Properties& prop = getProperties();

  coil::stringTo(m_dt, prop["pdservo.dt"].c_str());
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
  // set PD gain
  //
  coil::stringTo(m_pGain, prop["pdservo.pgain"].c_str());
  coil::stringTo(m_dGain, prop["pdservo.dgain"].c_str());

  m_dof = m_pGain.size();
  std::cout << m_instanceName << " : dof = " << m_dof << std::endl;


  if( m_pGain.size() != m_dGain.size() ) {
    std::cerr << m_instanceName << " : failed to load gain" << std::endl;
    return RTC::RTC_ERROR;
  }
  else {
    std::cout << m_instanceName << " : p gain =";  for( int i = 0; i < m_dof; i++)  std::cout << "  " << m_pGain[i];  std::cout << std::endl;
    std::cout << m_instanceName << " : d gain =";  for( int i = 0; i < m_dof; i++)  std::cout << "  " << m_dGain[i];  std::cout << std::endl;

  }

  m_qPre.resize(m_dof);
  m_qRefPre.resize(m_dof);

  m_qCur.data.length(m_dof);
  m_qRef.data.length(m_dof);
  m_tauRef.data.length(m_dof);

  return RTC::RTC_OK;
}

double tcount(0.0);
RTC::ReturnCode_t creekPdServo::onActivated(RTC::UniqueId ec_id)
{
  tcount = 0.0;
  std::cout << m_instanceName << " : onActivated" << std::endl;

  //
  // set initial data
  //
  if( m_qCurIn.isNew() ) {
    m_qCurIn.read();

    for(int i = 0; i < m_dof; i++) {
      m_qPre[i]      = m_qCur.data[i];
      m_qRefPre[i]   = m_qCur.data[i];
      m_qRef.data[i] = m_qCur.data[i];
    }
    std::cout << "servo : q0 = " << m_qCur.data[0] << std::endl;
  }
  else {
    std::cout << m_instanceName << " : connection error" << std::endl;
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPdServo::onExecute(RTC::UniqueId ec_id)
{
  tcount+=m_dt;
  //std::cout << "servo : time = " << tcount << std::endl;

  if( m_qCurIn.isNew() )  m_qCurIn.read();
  if( m_qRefIn.isNew() )  m_qRefIn.read();
  
  for( int i = 0; i < m_dof; i++) {
    double qCur = m_qCur.data[i];
    double qRef = m_qRef.data[i];
    double vCur = (qCur - m_qPre[i]) / m_dt;
    double vRef = (qRef - m_qRefPre[i]) / m_dt;

    m_tauRef.data[i] = m_pGain[i] * (qRef - qCur) + m_dGain[i] * (vRef - vCur);

    m_qPre[i]    = m_qCur.data[i];
    m_qRefPre[i] = m_qRef.data[i];
  }
  m_tauRefOut.write();
  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------


extern "C"
{
  void creekPdServoInit(RTC::Manager * manager)
  {
    coil::Properties profile(module_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekPdServo>,
                             RTC::Delete<creekPdServo>);
  }
};
