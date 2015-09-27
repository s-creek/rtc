// -*- C++ -*-
/*!
 * @file  creekQrCodeReader.cpp * @brief creekQrCodeReader * $Date$ 
 *
 * $Id$ 
 */
#include "creekQrCodeReader.h"

// Module specification
static const char* creekqrcodereader_spec[] =
  {
    "implementation_id", "creekQrCodeReader",
    "type_name",         "creekQrCodeReader",
    "description",       "creekQrCodeReader",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekQrCodeReader",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekQrCodeReader::creekQrCodeReader(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_imageIn("image", m_image),
    m_creekQrCodeReaderServicePort("creekQrCodeReaderService")
{
}

creekQrCodeReader::~creekQrCodeReader()
{
}


RTC::ReturnCode_t creekQrCodeReader::onInitialize()
{
  std::cout << "creekQrCodeReader : onInitialize" << std::endl;
  addInPort("image", m_imageIn);

  m_creekQrCodeReaderServicePort.registerProvider("service0", "creekQrCodeReaderService", m_service0);
  addPort(m_creekQrCodeReaderServicePort);

  
  // init zbar
  m_zbar.set_format("Y800");
  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekQrCodeReader::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekQrCodeReader : onActivated" << std::endl;
  if( m_imageIn.isNew() ) {
    if( !updateImage() )
      return RTC::RTC_ERROR;
 
    std::cout << "QrCodeReader : image data" << std::endl;
    std::cout << "  size  : " << m_image.data.image.width << " x " <<  m_image.data.image.height << std::endl;
    std::cout << "  data  : " << m_image.data.image.raw_data.length() << std::endl;
    std::cout << "  format: " << m_image.data.image.format << std::endl;
  }
  cv::namedWindow("QrCodeReader:gray", CV_WINDOW_AUTOSIZE);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekQrCodeReader::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekQrCodeReader : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekQrCodeReader::onExecute(RTC::UniqueId ec_id)
{
  if( m_imageIn.isNew() ) {
    if( !updateImage() )
      return RTC::RTC_ERROR;
    //std::cout << (m_frame.total()*m_frame.channels()) << " " << m_image.data.image.raw_data.length() << std::endl;
 

    if( m_scanner.scan(m_zbar) != 0 ) {
      for(zbar::Image::SymbolIterator symbol = m_zbar.symbol_begin(); symbol != m_zbar.symbol_end(); ++symbol) {
	if( symbol->get_type() == zbar::ZBAR_QRCODE )
	std::cout << "decoded data : " << symbol->get_data() << std::endl;
   
      }
    }


    cv::cvtColor(m_frame, m_frame, CV_RGB2BGR);
    cv::imshow("QrCodeReader:image", m_frame);
    cv::imshow("QrCodeReader:gray", m_gray);
    //cv::waitKey(1);

    return RTC::RTC_OK;
  }
}


bool creekQrCodeReader::updateImage()
{
  // get new buffer
  m_imageIn.read();


  // size check
  int a = m_frame.total()*m_frame.channels();
  int b = m_image.data.image.raw_data.length();
  if( a != b ) {
    switch(m_image.data.image.format)
      {
      case Img::CF_GRAY:
	m_frame = cv::Mat::zeros(m_image.data.image.height, m_image.data.image.width, CV_8UC1);
	break;
      case Img::CF_RGB:
	m_frame = cv::Mat::zeros(m_image.data.image.height, m_image.data.image.width, CV_8UC3);
	break;
      default:
	return false;
      }

    std::cout << "QrCodeReader : resize image data (opecv)" << std::endl;
  }


  // copy original image
  memcpy(m_frame.data,
	 m_image.data.image.raw_data.get_buffer(),
	 m_image.data.image.raw_data.length());


  // convert image
  switch(m_image.data.image.format)
    {
    case Img::CF_GRAY:
      m_gray = m_frame;
      break;
    case Img::CF_RGB:
      cv::cvtColor(m_frame, m_gray, CV_RGB2GRAY);
      break;
    default:
      return false;
    }


  // size check (zbar)
  if( m_gray.total() != m_zbar.get_data_length() ) {
    m_zbar.set_size(m_gray.size().width, m_gray.size().height);
    m_zbar.set_data(m_gray.data, m_gray.total());
    std::cout << "QrCodeReader : resize image data (zbar)" << std::endl;
  }


  return true;
}


extern "C"
{
 
  void creekQrCodeReaderInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekqrcodereader_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekQrCodeReader>,
                             RTC::Delete<creekQrCodeReader>);
  }
  
};



