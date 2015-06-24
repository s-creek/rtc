// -*- c++ -*-

// dependent
//   OpenCV 2.2.0
//   ARToolKitPlus 2.3.0
//   boost 1.53.0

#ifndef IMAGE_INTERFACE_TYPES_H
#define IMAGE_INTERFACE_TYPES_H

// rtm
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>
//#include <rtm/DataOutPort.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// ARToolKitPlus
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <ARToolKitPlus/TrackerMultiMarker.h>


class ImageInterfaceTypes : public RTC::TimedLongSeq
{
 public:
  ImageInterfaceTypes(std::string in_name="");
  ImageInterfaceTypes(unsigned int in_xsize, unsigned int in_ysize);
  ~ImageInterfaceTypes();

  void setPort( RTC::InPort<RTC::TimedLongSeq> *in_port ) {
	m_port = in_port;
  }

  void set(unsigned int in_xsize, unsigned int in_ysize, RTC::InPort<RTC::TimedLongSeq> *in_port=NULL);
  bool set(unsigned int in_xsize, unsigned int in_ysize, std::string &in_camera_file, std::string &in_patt_file, bool template_mode=false, RTC::InPort<RTC::TimedLongSeq> *in_port=NULL);
  bool update();
  void getConf(double *in_conf);

  
 private:
  unsigned int m_xsize, m_ysize;
  IplImage* m_image_cv;

  //ARToolKitPlus::TrackerSingleMarker *m_tracker;
  ARToolKitPlus::TrackerMultiMarker *m_tracker;

  std::string m_name;
  RTC::InPort<RTC::TimedLongSeq> *m_port;

  bool m_showImage;
};


#endif
