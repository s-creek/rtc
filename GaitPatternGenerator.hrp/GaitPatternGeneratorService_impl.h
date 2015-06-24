// -*-C++-*-

#include "GaitPatternGeneratorService.hh"

#ifndef GAIT_PATTERN_GENERATOR_SERVICE_IMPL_H
#define GAIT_PATTERN_GENERATOR_SERVICE_IMPL_H

class GaitPatternGenerator;

class GaitPatternGeneratorService_impl
  : public virtual POA_OpenHRP::GaitPatternGeneratorService,
	public virtual PortableServer::RefCountServantBase
{
public:
  GaitPatternGeneratorService_impl();
  virtual ~GaitPatternGeneratorService_impl();

  void setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time);
  void stop();
  
  void setComponent (GaitPatternGenerator * i_comp) {
    m_comp = i_comp;
  }

private:
  GaitPatternGenerator * m_comp;
};

#endif // GAIT_PATTERN_GENERATOR_SERVICE_IMPL_H
