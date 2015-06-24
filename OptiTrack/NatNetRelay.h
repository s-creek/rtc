// -*- C++ -*-

#ifndef NATNETRELAY_H
#define NATNETRELAY_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <NatNetTypes.h>
#include <NatNetClient.h>

using namespace RTC;

class NatNetRelay  : public RTC::DataFlowComponentBase
{
public:
	NatNetRelay(RTC::Manager* manager);
	~NatNetRelay();
	
	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);


protected:
	// Dataport
	TimedDoubleSeq          m_dataStream;
	OutPort<TimedDoubleSeq> m_dataStreamOut;


	// Configuration variable declaration
	int          m_connectionType;
	std::string  m_serverIpAddress;
	std::string  m_clientIpAddress;



private:
	int CreateClient(int inConnectionType);

	NatNetClient* m_theClient;
};


extern "C"
{
	DLL_EXPORT void NatNetRelayInit(RTC::Manager* manager);
};

#endif // NATNETRELAY_H

