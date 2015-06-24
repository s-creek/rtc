// -*- c++ -*-

#include "ImageInterfaceTypes.h"


ImageInterfaceTypes::ImageInterfaceTypes(std::string in_name)
  : m_image_cv(NULL),
    m_port(NULL),
    m_tracker(NULL),
    m_xsize(0),
    m_ysize(0),
    m_name(in_name),
    m_showImage(true)
{

}


ImageInterfaceTypes::ImageInterfaceTypes(unsigned int in_xsize, unsigned int in_ysize)
  : m_image_cv(NULL),
    m_port(NULL),
    m_tracker(NULL)
{
  set(in_xsize, in_ysize);
}


ImageInterfaceTypes::~ImageInterfaceTypes()
{
  if( m_port && m_showImage )
    cvDestroyWindow(m_name.c_str());
  
  if( m_image_cv ) {
    cvReleaseImage(&m_image_cv);
    m_image_cv = NULL;
  }

  if( m_tracker ) {
    delete m_tracker;
    m_tracker = NULL;
  }
}


void ImageInterfaceTypes::set(unsigned int in_xsize, unsigned int in_ysize, RTC::InPort<RTC::TimedLongSeq> *in_port)
{
  m_xsize = in_xsize;
  m_ysize = in_ysize;
  
  data.length(m_xsize*m_ysize);

  if( m_image_cv ) {
    cvReleaseImage(&m_image_cv);
    m_image_cv = NULL;
  }
  //m_image_cv = cvCreateImage( cvSize(m_xsize, m_ysize), IPL_DEPTH_8U, 3 );
  m_image_cv = cvCreateImage( cvSize(m_xsize, m_ysize), IPL_DEPTH_8U, 4 );


  if( in_port ) {
    m_port = in_port;
    m_name = m_name + ":" + m_port->name();
  }
}


bool ImageInterfaceTypes::set(unsigned int in_xsize, unsigned int in_ysize, std::string &in_camera_file, std::string &in_patt_file, bool template_mode, RTC::InPort<RTC::TimedLongSeq> *in_port)
{
  set(in_xsize, in_ysize, in_port);
  
 
  if( m_tracker ) {
    delete m_tracker;
    m_tracker = NULL;
  }

  
  if( template_mode ) {
    m_tracker = new ARToolKitPlus::TrackerMultiMarker(m_xsize, m_ysize, 8, 16, 16, 16, 10);
    m_tracker->setMarkerMode(ARToolKitPlus::MARKER_TEMPLATE);
  }
  else {
    m_tracker = new ARToolKitPlus::TrackerMultiMarker(m_xsize, m_ysize, 8, 6, 6, 6, 3);
    m_tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
  }


  if( !m_tracker->init(in_camera_file.c_str(), in_patt_file.c_str(), 1.0f, 1000.0f) ) {
    printf("ERROR: init() failed\n");

    delete m_tracker;
    m_tracker = NULL;
	
    return false;
  }
  

  //m_tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_BGR);
  m_tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_BGRA);
  m_tracker->setThreshold(150);
  m_tracker->activateAutoThreshold(true);
  m_tracker->setBorderWidth(0.25f);


  m_tracker->getCamera()->printSettings();


  return true;
}


bool ImageInterfaceTypes::update()
{
  bool found(false);
  
  if( m_port ) {
    m_port->read();

 
    if( data.length() != m_xsize*m_ysize )
      return found;


    //
    // convert image data
    //
    long tmp;
    char imA, imR, imG, imB;

    for(unsigned int i = 0; i < m_ysize; i++) {
      for(unsigned int j = 0; j < m_xsize; j++) {
	tmp = data[i*m_xsize+j];

	imB = tmp & 0xff;   tmp = tmp >> 8;
	imG = tmp & 0xff;   tmp = tmp >> 8;
	imR = tmp & 0xff;   tmp = tmp >> 8;
	imA = tmp & 0xff;

	/*
	// ViewSimulation (ARGB) -> OpenCV (BGR)
	m_image_cv->imageData[(i*m_xsize+j)*3 + 0] = imB;
	m_image_cv->imageData[(i*m_xsize+j)*3 + 1] = imG;
	m_image_cv->imageData[(i*m_xsize+j)*3 + 2] = imR;
	*/

	// ViewSimulation (ARGB) -> OpenCV (BGRA)
	m_image_cv->imageData[(i*m_xsize+j)*4 + 0] = imB;
	m_image_cv->imageData[(i*m_xsize+j)*4 + 1] = imG;
	m_image_cv->imageData[(i*m_xsize+j)*4 + 2] = imR;
	m_image_cv->imageData[(i*m_xsize+j)*4 + 3] = imA;
      }
    }


    //
    // detect marker
    //
    if( m_tracker ) {
      int numDetected = m_tracker->calc( reinterpret_cast<uint8_t*>(m_image_cv->imageData) );
      if( numDetected > 0 ) {

	ARFloat trans[3][4];
	m_tracker->getARMatrix( trans );

	for(int i = 0; i < 3; i++) {
	  if( abs(trans[i][3]) > 1.0e-6 )
	    found = true;
	}

		
	//
	// write line
	//
	if(m_showImage) {
	  for(int i = 0; i < numDetected; i++) {
	    ARToolKitPlus::ARMarkerInfo marker = m_tracker->getDetectedMarker(i);

	    int dir = marker.dir;
		
	    double vertex[4][2];
	    for(int j = 0; j < 4; j++)
	      for(int k = 0; k < 2; k++)
		vertex[j][k] = marker.vertex[j][k];


	    // write line
	    cvLine(m_image_cv, cvPoint(vertex[(4-dir)%4][0], vertex[(4-dir)%4][1]), cvPoint(vertex[(5-dir)%4][0], vertex[(5-dir)%4][1]), CV_RGB(255,0,0),   3);
	    cvLine(m_image_cv, cvPoint(vertex[(5-dir)%4][0], vertex[(5-dir)%4][1]), cvPoint(vertex[(6-dir)%4][0], vertex[(6-dir)%4][1]), CV_RGB(0,255,0),   3);
	    cvLine(m_image_cv, cvPoint(vertex[(6-dir)%4][0], vertex[(6-dir)%4][1]), cvPoint(vertex[(7-dir)%4][0], vertex[(7-dir)%4][1]), CV_RGB(0,0,255),   3);
	    cvLine(m_image_cv, cvPoint(vertex[(7-dir)%4][0], vertex[(7-dir)%4][1]), cvPoint(vertex[(4-dir)%4][0], vertex[(4-dir)%4][1]), CV_RGB(255,255,0), 3);
	  }
	} // end of write line
      }
    }


    if(m_showImage)
      cvShowImage(m_name.c_str(), m_image_cv);
    cvWaitKey(5);
  }

  return found;
}


void ImageInterfaceTypes::getConf(double *in_conf)
{
  ARFloat trans[3][4];
  
  m_tracker->getARMatrix( trans );

  /*
    ARFloat center[2] = {0};
    ARToolKitPlus::ARMarkerInfo marker = m_tracker->getDetectedMarker(0);
    m_tracker->executeSingleMarkerPoseEstimator(&marker, center, 80.0, trans);
  */
  
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      in_conf[j*3+k] = trans[j][k];
    }
    in_conf[9+j] = trans[j][3] / 1000.0;  // [mm] -> [m]
  }
}
