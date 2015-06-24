// -*-C++-*-
/*!
 * @file  ArmControlCartesianService_impl.cpp
 * @brief Service implementation code of ArmControlCartesianService.idl
 *
 */

#include "ArmControlCartesianService_impl.h"
#include "ArmControlCartesian.h"


ArmControlCartesianService_impl::ArmControlCartesianService_impl()
{
}


ArmControlCartesianService_impl::~ArmControlCartesianService_impl()
{
}


::CORBA::Boolean ArmControlCartesianService_impl::isEmpty()
{
  return m_comp->isEmpty();
}


::CORBA::Boolean ArmControlCartesianService_impl::setTargetAngular(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double r, ::CORBA::Double p, ::CORBA::Double w, ::CORBA::Double time)
{
  return m_comp->setTargetAngular(x,y,z,r,p,w,time); 
}


void ArmControlCartesianService_impl::getCurrentConfiguration(::CORBA::Double& x, ::CORBA::Double& y, ::CORBA::Double& z, ::CORBA::Double& r, ::CORBA::Double& p, ::CORBA::Double& w)
{
  m_comp->getCurrentConfiguration(x,y,z,r,p,w);
}


void ArmControlCartesianService_impl::emergencyStop()
{
  m_comp->emergencyStop();
}



// End of example implementational code



