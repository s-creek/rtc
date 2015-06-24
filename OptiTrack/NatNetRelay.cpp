// -*- C++ -*-

#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>

#include "NatNetRelay.h"

void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages


#include <vector>
bool m_wait(true);
std::vector<double> m_outputData;  // [id, x, y, z, R] x n
void setOutputData(int id, double x, double y, double z, double qx, double qy, double qz, double qw);


// Module specification
static const char* natnetrelay_spec[] =
  {
    "implementation_id", "NatNetRelay",
    "type_name",         "NatNetRelay",
    "description",       "Relay component for Motive",
    "version",           "1.0.0",
    "vendor",            "skj",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
	//"conf.default.server_ip_address", "192.168.100.121",
""
  };


NatNetRelay::NatNetRelay(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
	m_dataStreamOut("dataStream", m_dataStream),
	m_connectionType(ConnectionType_Multicast),
	m_serverIpAddress("localhost"),
	m_clientIpAddress("localhost"),
	m_theClient(NULL)
{
}


NatNetRelay::~NatNetRelay()
{
}


RTC::ReturnCode_t NatNetRelay::onInitialize()
{
	printf("------------------------------------------------------------------\n");
	printf("onInitialize\n");
	printf("------------------------------------------------------------------\n");


	addOutPort("dataStream", m_dataStreamOut);


	// Bind variables and configuration variable
	bindParameter("connection_type",   m_connectionType,  "0");
	bindParameter("server_ip_address", m_serverIpAddress, "localhost");
	bindParameter("client_ip_address", m_clientIpAddress, "localhost");


	m_dataStream.data.length(0);


	return RTC::RTC_OK;	
}


RTC::ReturnCode_t NatNetRelay::onActivated(RTC::UniqueId ec_id)
{
	printf("------------------------------------------------------------------\n");
	printf("onActivated\n");
	printf("------------------------------------------------------------------\n");


	// set configuration
	if( (m_connectionType != ConnectionType_Multicast) && (m_connectionType != ConnectionType_Unicast) )
	{
		printf("[NatNetRelay] set connection type error\n");
		return RTC::RTC_ERROR;
	}

	if( strcmp(m_serverIpAddress.c_str(), "localhost") == 0 )
	{
		m_serverIpAddress = "";
	}

	if( strcmp(m_clientIpAddress.c_str(), "localhost") == 0 )
	{
		m_clientIpAddress = "";
	}


	int retCode;
	// Create NatNet Client
    retCode = CreateClient(m_connectionType);
    if(retCode != ErrorCode_OK)
    {
        printf("[NatNetRelay] Error initializing client.  See log for details.  Exiting\n");
        return RTC::RTC_ERROR;
    }
    else
    {
        printf("[NatNetRelay] Client initialized and ready.\n");
    }


	// send/receive test request
	printf("[NatNetRelay] Sending Test Request\n");
	void* response;
	int nBytes;
	retCode = m_theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (retCode == ErrorCode_OK)
	{
		printf("[NatNetRelay] Received: %s\n", (char*)response);
	}


	// Retrieve Data Descriptions from server
	printf("[NatNetRelay] Requesting Data Descriptions...\n");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = m_theClient->GetDataDescriptions(&pDataDefs);
	if(!pDataDefs)
	{
		printf("[NatNetRelay] Unable to retrieve Data Descriptions.\n");
	}
	else
	{
        printf("[NatNetRelay] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("\tMarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("\t%s\n", pMS->szMarkerNames[i]);

            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("\tRigidBody Name : %s\n", pRB->szName);
                printf("\tRigidBody ID : %d\n", pRB->ID);
                printf("\tRigidBody Parent ID : %d\n", pRB->parentID);
                printf("\tParent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("\tSkeleton Name : %s\n", pSK->szName);
                printf("\tSkeleton ID : %d\n", pSK->skeletonID);
                printf("\tRigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    printf("\t  RigidBody Name : %s\n", pRB->szName);
                    printf("\t  RigidBody ID : %d\n", pRB->ID);
                    printf("\t  RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("\t  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
			printf("\n");
        }      
	}


	return RTC::RTC_OK;
}


RTC::ReturnCode_t NatNetRelay::onExecute(RTC::UniqueId ec_id)
{
	if(m_wait)
		return RTC::RTC_OK;


	m_dataStream.data.length(m_outputData.size());
	for(unsigned int i = 0; i < m_outputData.size(); i++)
		m_dataStream.data[i] = m_outputData.at(i);
	m_dataStreamOut.write();


	printf("[NatNetRelay] output\n");
	m_wait = true;


	return RTC::RTC_OK;
}


RTC::ReturnCode_t NatNetRelay::onDeactivated(RTC::UniqueId ec_id)
{
	printf("------------------------------------------------------------------\n");
	printf("onDeactivated\n");
	printf("------------------------------------------------------------------\n");


	m_theClient->Uninitialize();


	return RTC::RTC_OK;
}


//-----------------------------------------------------------------------------------------


int NatNetRelay::CreateClient(int inConnectionType)
{
    // release previous server
    if(m_theClient)
    {
        m_theClient->Uninitialize();
        delete m_theClient;
    }
    // create NatNet client
    m_theClient = new NatNetClient(inConnectionType);


    // Set callback handlers
	//m_theClient->SetMessageCallback(MessageHandler);
    m_theClient->SetVerbosityLevel(Verbosity_Debug);
	m_theClient->SetDataCallback( DataHandler, m_theClient );	// this function will receive data from the server


    // Init Client and connect to NatNet server
    // to use NatNet default port assigments
	char clientIpAddress[128];  strcpy(clientIpAddress, m_clientIpAddress.c_str());  // set client IP address
	char serverIpAddress[128];  strcpy(serverIpAddress, m_serverIpAddress.c_str());  // set server IP address
    int retCode = m_theClient->Initialize(clientIpAddress, serverIpAddress);   
	if(retCode != ErrorCode_OK)
    {
        printf("[NatNetRelay] Unable to connect to server.  Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // print server info
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        m_theClient->GetServerDescription(&ServerDescription);
        if(!ServerDescription.HostPresent)
        {
            printf("[NatNetRelay] Unable to connect to server. Host not present. Exiting.\n");
            return ErrorCode_Internal;
        }
        printf("[NatNetRelay] Server application info:\n");
        printf("\tApplication: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
        printf("\tNatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
            ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
        printf("\tClient IP: %s\n", clientIpAddress);
        printf("\tServer IP: %s\n", serverIpAddress);
        printf("\tServer Name: %s\n\n", ServerDescription.szHostComputerName);	
    }

    return ErrorCode_OK;
}


//-----------------------------------------------------------------------------------------
// callback handlers
//-----------------------------------------------------------------------------------------

// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*) pUserData;

	
	printf("------------------------------------------------------------------\n");
	printf("Received frame %d\n", data->iFrame);


	if( !m_wait ) {
		printf("[NatNetRelay] now writing\n");
		return;
	}
	m_outputData.clear();


	int i=0;
	
	/*
    // same system latency test
    float fThisTick = (float)GetTickCount();
    float fDiff = fThisTick - data->fLatency;
    double dDuration = fDiff;
    printf("Latency (same system) (msecs): %3.2lf\n", dDuration);


	// timecode
	// decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	printf("Timecode : %s\n", szTimecode);

	// Other Markers
	printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	for(i=0; i < data->nOtherMarkers; i++)
	{
		printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n",
			i,
			data->OtherMarkers[i][0],
			data->OtherMarkers[i][1],
			data->OtherMarkers[i][2]);
	}
	*/

	// Rigid Bodies
	printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(i=0; i < data->nRigidBodies; i++)
	{
		printf("Rigid Body [ID=%d  Error=%3.2f]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError);
		printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
			data->RigidBodies[i].x,
			data->RigidBodies[i].y,
			data->RigidBodies[i].z,
			data->RigidBodies[i].qx,
			data->RigidBodies[i].qy,
			data->RigidBodies[i].qz,
			data->RigidBodies[i].qw);


		// set output data
		setOutputData(data->RigidBodies[i].ID, data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
			data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw);


		printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
		for(int iMarker=0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
		{
            printf("\t\t");
            if(data->RigidBodies[i].MarkerIDs)
                printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
            if(data->RigidBodies[i].MarkerSizes)
                printf("\tMarkerSize:%3.2f", data->RigidBodies[i].MarkerSizes[iMarker]);
            if(data->RigidBodies[i].Markers)
                printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n" ,
                    data->RigidBodies[i].Markers[iMarker][0],
                    data->RigidBodies[i].Markers[iMarker][1],
                    data->RigidBodies[i].Markers[iMarker][2]);
        }
	}

	/*
	// skeletons
	printf("Skeletons [Count=%d]\n", data->nSkeletons);
	for(i=0; i < data->nSkeletons; i++)
	{
		sSkeletonData skData = data->Skeletons[i];
		printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
		for(int j=0; j< skData.nRigidBodies; j++)
		{
			sRigidBodyData rbData = skData.RigidBodyData[j];
			printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw );

			printf("\tRigid body markers [Count=%d]\n", rbData.nMarkers);
			for(int iMarker=0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
			{
				printf("\t\t");
				if(rbData.MarkerIDs)
					printf("MarkerID:%d", rbData.MarkerIDs[iMarker]);
				if(rbData.MarkerSizes)
					printf("\tMarkerSize:%3.2f", rbData.MarkerSizes[iMarker]);
				if(rbData.Markers)
					printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n" ,
					data->RigidBodies[i].Markers[iMarker][0],
					data->RigidBodies[i].Markers[iMarker][1],
					data->RigidBodies[i].Markers[iMarker][2]);
			}
		}
	}

	// labeled markers
	printf("Labeled Markers [Count=%d]\n", data->nLabeledMarkers);
	for(i=0; i < data->nLabeledMarkers; i++)
	{
		sMarker marker = data->LabeledMarkers[i];
		printf("Labeled Marker [ID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n", marker.ID, marker.size, marker.x, marker.y, marker.z);
	}
	*/
	m_wait = false;
}


// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}


void setOutputData(int id, double x, double y, double z, double qx, double qy, double qz, double qw)
{
	/*
	double R[] = {
		1-2*(qy*qy+qz*qz),  2*(qx*qy+qz*qw),    2*(qx*qz-qy*qw),
		2*(qx*qy-qz*qw),    1-2*(qz*qz+qx*qx),  2*(qy*qz+qx*qz),
		2*(qx*qz+qy*qw),    2*(qy*qz-qx*qz),    1-2*(qx*qx+qy*qy)
	};
	*/
	/*
	m_outputData.push_back(id);
	m_outputData.push_back(x);
	m_outputData.push_back(y);
	m_outputData.push_back(z);
	*/


	// coordinate transform
	double R[] = {
		1-2*(qx*qx+qy*qy),  2*(qx*qz-qy*qw),    2*(qy*qz+qx*qz),
		2*(qx*qz+qy*qw),    1-2*(qy*qy+qz*qz),  2*(qx*qy-qz*qw),
		2*(qy*qz-qx*qz),    2*(qx*qy+qz*qw),    1-2*(qz*qz+qx*qx)
	};

	m_outputData.push_back(id);
	m_outputData.push_back(z);  // coordinate transform
	m_outputData.push_back(x);  // coordinate transform
	m_outputData.push_back(y);  // coordinate transform

	for(int i = 0; i < 9; i++)
		m_outputData.push_back(R[i]);

	/*
	// check data
	double norm(qx*qx+qy*qy+qz*qz+qw*qw);
	printf("norm = %5.3f\nR =\n", norm);
	for(int i = 0; i < 3; i++)
		printf("  %5.3f  %5.3f  %5.3f\n", R[i*3], R[i*3+1], R[i*3+2]);
	*/
}


//-----------------------------------------------------------------------------------------


extern "C"
{
	void NatNetRelayInit(RTC::Manager* manager)
	{
		coil::Properties profile(natnetrelay_spec);
		manager->registerFactory(profile,
			RTC::Create<NatNetRelay>,
			RTC::Delete<NatNetRelay>);
	}
};



