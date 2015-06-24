// -*-C++-*-
/*!
 * @file  ArmControlCartesianService_impl.h
 * @brief Service implementation header of ArmControlCartesianService.idl
 *
 */

#include "ArmControlCartesianService.hh"


#ifndef ARMCONTROLCARTESIANSERVICE_IMPL_H
#define ARMCONTROLCARTESIANSERVICE_IMPL_H
 
class ArmControlCartesian;

class ArmControlCartesianService_impl
  : public virtual POA_OpenHRP::ArmControlCartesianService,
    public virtual PortableServer::RefCountServantBase
{
public:
  // standard constructor
  ArmControlCartesianService_impl();
  virtual ~ArmControlCartesianService_impl();

  // attributes and operations
  ::CORBA::Boolean isEmpty();
  ::CORBA::Boolean setTargetAngular(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double r, ::CORBA::Double p, ::CORBA::Double w, ::CORBA::Double time);
  void getCurrentConfiguration(::CORBA::Double& x, ::CORBA::Double& y, ::CORBA::Double& z, ::CORBA::Double& r, ::CORBA::Double& p, ::CORBA::Double& w);
  void emergencyStop();


  void setComponent (ArmControlCartesian * i_comp) {
    m_comp = i_comp;
  }

private:
  ArmControlCartesian * m_comp;
};



#endif // ARMCONTROLCARTESIANSERVICE_IMPL_H


