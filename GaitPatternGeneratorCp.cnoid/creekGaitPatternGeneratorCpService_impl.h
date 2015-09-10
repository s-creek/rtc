// -*-C++-*-

#include "creekGaitPatternGeneratorCpService.hh"


#ifndef CREEKGAITPATTERNGENERATORCPSERVICE_IMPL_H
#define CREEKGAITPATTERNGENERATORCPSERVICE_IMPL_H

class creekGaitPatternGeneratorCp;

class creekGaitPatternGeneratorCpService_impl
 : public virtual POA_OpenHRP::creekGaitPatternGeneratorCpService,
   public virtual PortableServer::RefCountServantBase
{
public:
  creekGaitPatternGeneratorCpService_impl();
  virtual ~creekGaitPatternGeneratorCpService_impl();

  void setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time);
  void startStepping();
  void stopStepping();
  void test();

  void setComponent (creekGaitPatternGeneratorCp * i_comp) {
    m_comp = i_comp;
  }

private:
  creekGaitPatternGeneratorCp *m_comp;
};

#endif // CREEKGAITPATTERNGENERATORCPSERVICE_IMPL_H


