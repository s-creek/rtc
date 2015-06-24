// -*- c++ -*-

#include "SingleImageProcessing.h"

#include "VectorConvert.h"

// Module specification
static const char* module_spec[] =
  {
    "implementation_id", "SingleImageProcessing",
    "type_name",         "SingleImageProcessing",
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


SingleImageProcessing::SingleImageProcessing(RTC::Manager* manager)
  :  RTC::DataFlowComponentBase(manager),
     m_targetConfOut("targetConfOut", m_targetConf)
{
}


RTC::ReturnCode_t SingleImageProcessing::onInitialize()
{
  // get properties
  RTC::Properties& prop = getProperties();
  m_instanceName = prop["instance_name"];

  
  m_image   = new ImageInterfaceTypes(m_instanceName);
  m_imageIn = new RTC::InPort<RTC::TimedLongSeq>("imageIn", *m_image);

	
  // Set InPort/OutPort buffers
  addInPort("imgaeIn", *m_imageIn);
  addOutPort("targetConfOut", m_targetConfOut);


  std::vector<int> image_size(2);  image_size[0] = 640;  image_size[1] = 480;
  coil::stringTo(image_size, prop[m_instanceName+".image_size"].c_str());
  

  bool template_mode(false), ret;
  if(template_mode) {
    ret = m_image->set(image_size[0], image_size[1], prop[m_instanceName+".camera_file"], prop[m_instanceName+".patt_file"], true, m_imageIn);
  }
  else{
    ret = m_image->set(image_size[0], image_size[1], prop[m_instanceName+".camera_file"], prop[m_instanceName+".patt_file"], false, m_imageIn);
  }
  if(ret) {
    std::cout << m_instanceName << " : image size = " << image_size[0] << " " << image_size[1] << std::endl;
  }
  else {
    std::cerr << m_instanceName << " : image data init error" << std::endl;
    return RTC::RTC_ERROR;
  }
	

  // init OutPort
  m_targetConf.data.length(12);

  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SingleImageProcessing::onExecute(RTC::UniqueId ec_id)
{
  if( m_imageIn->isNew() ) {

    if( m_image->update() ) {
      std::cout << "update" << std::endl;
      m_image->getConf( &m_targetConf.data[0] );
      m_targetConfOut.write();
    }
  }
  
  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------


extern "C"
{
 
  void SingleImageProcessingInit(RTC::Manager* manager)
  {
    coil::Properties profile(module_spec);
    manager->registerFactory(profile,
                             RTC::Create<SingleImageProcessing>,
                             RTC::Delete<SingleImageProcessing>);
  }
  
};
