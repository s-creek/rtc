// -*-C++-*-

#include "creekGaitPatternGeneratorCpService_impl.h"
#include "creekGaitPatternGeneratorCp.h"

creekGaitPatternGeneratorCpService_impl::creekGaitPatternGeneratorCpService_impl()
  : m_comp(NULL)
{
}

creekGaitPatternGeneratorCpService_impl::~creekGaitPatternGeneratorCpService_impl()
{
}

void creekGaitPatternGeneratorCpService_impl::setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time)
{
}

void creekGaitPatternGeneratorCpService_impl::startStepping()
{
  if( m_comp )
    m_comp->startStepping();
}

void creekGaitPatternGeneratorCpService_impl::stopStepping()
{
  if( m_comp )
    m_comp->stopStepping();
}

void creekGaitPatternGeneratorCpService_impl::test()
{
  if( m_comp )
    m_comp->test();
}



// End of example implementational code



