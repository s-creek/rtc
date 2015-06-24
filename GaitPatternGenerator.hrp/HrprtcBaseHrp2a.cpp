// -*- C++ -*-

#include "HrprtcBaseHrp2a.h"

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/ModelLoaderUtil.h>


// Module specification
static const char* hrprtcbasehrp2a_spec[] =
{
    "implementation_id", "HrprtcBaseHrp2a",
    "type_name",         "HrprtcBaseHrp2a",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "General Robotix,Inc.",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
};


HrprtcBaseHrp2a::HrprtcBaseHrp2a(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_jointDatIn("jointDatIn", m_jointDat),
    m_jointDatOut("jointDatOut", m_jointDat),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_accRefOut("accRef", m_accRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_robot(NULL)
{
}


HrprtcBaseHrp2a::~HrprtcBaseHrp2a()
{
}


RTC::ReturnCode_t HrprtcBaseHrp2a::onInitialize()
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


 
  //
  // get properties
  //
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
  // set initial data
  //
  unsigned int dof = m_robot->numJoints();
  m_jointDat.data.length(dof);
  for(unsigned int i = 0; i < dof; i++) m_jointDat.data[i] = 0.0;
  

  m_basePosInit.data.length(3);
  m_baseRpyInit.data.length(3);
  
  m_zmpRef.data.length(3);
  m_accRef.data.length(3);
  m_basePos.data.length(3);
  m_baseRpy.data.length(3);
  
  for(int i = 0; i < 3; i++) {
	m_basePosInit.data[i] = 0.0;
	m_baseRpyInit.data[i] = 0.0;
	
	m_zmpRef.data[i] = 0.0;
	m_accRef.data[i] = 0.0;
    m_basePos.data[i] = 0.0;
    m_baseRpy.data[i] = 0.0;
  }

  
  return RTC::RTC_OK;
}
