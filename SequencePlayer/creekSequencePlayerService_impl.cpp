// -*-C++-*-

#include "creekSequencePlayerService_impl.h"
#include "creekSequencePlayer.h"

/*
 * Example implementational code for IDL interface OpenHRP::creekSequencePlayerService
 */
creekSequencePlayerService_impl::creekSequencePlayerService_impl()
  : m_comp(NULL)
{
}


creekSequencePlayerService_impl::~creekSequencePlayerService_impl()
{
}


::CORBA::Boolean creekSequencePlayerService_impl::setJointAngles(const OpenHRP::dSequence& jvs, ::CORBA::Double tm)
{
  if( jvs.length() != m_comp->dof() ) {
    std::cout << "creekSequencePlayerService : error"<< std::endl;
    return false;
  }
  return m_comp->setJointAngles(jvs.get_buffer(), tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::setJointAngle(const char* jname, ::CORBA::Double jv, ::CORBA::Double tm)
{

}

::CORBA::Boolean creekSequencePlayerService_impl::setBasePos(const OpenHRP::dSequence& pos, ::CORBA::Double tm)
{

}

::CORBA::Boolean creekSequencePlayerService_impl::setBaseRpy(const OpenHRP::dSequence& rpy, ::CORBA::Double tm)
{
 
}

::CORBA::Boolean creekSequencePlayerService_impl::setZmp(const OpenHRP::dSequence& zmp, ::CORBA::Double tm)
{
 
}

::CORBA::Boolean creekSequencePlayerService_impl::isEmpty()
{
  return m_comp->isEmpty();
}



// End of example implementational code



