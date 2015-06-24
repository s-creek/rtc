// -*- c++ -*-

#include "GaitPatternGeneratorService_impl.h"
#include "GaitPatternGenerator.h"


GaitPatternGeneratorService_impl::GaitPatternGeneratorService_impl()
{
}


GaitPatternGeneratorService_impl::~GaitPatternGeneratorService_impl()
{
}

void GaitPatternGeneratorService_impl::setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time)
{
  m_comp->setTargetPos(x, y, th, time);
}


void GaitPatternGeneratorService_impl::stop()
{
  m_comp->stop();
}
