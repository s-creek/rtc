// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "creekGaitPatternGeneratorCpService.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



OpenHRP::creekGaitPatternGeneratorCpService_ptr OpenHRP::creekGaitPatternGeneratorCpService_Helper::_nil() {
  return ::OpenHRP::creekGaitPatternGeneratorCpService::_nil();
}

::CORBA::Boolean OpenHRP::creekGaitPatternGeneratorCpService_Helper::is_nil(::OpenHRP::creekGaitPatternGeneratorCpService_ptr p) {
  return ::CORBA::is_nil(p);

}

void OpenHRP::creekGaitPatternGeneratorCpService_Helper::release(::OpenHRP::creekGaitPatternGeneratorCpService_ptr p) {
  ::CORBA::release(p);
}

void OpenHRP::creekGaitPatternGeneratorCpService_Helper::marshalObjRef(::OpenHRP::creekGaitPatternGeneratorCpService_ptr obj, cdrStream& s) {
  ::OpenHRP::creekGaitPatternGeneratorCpService::_marshalObjRef(obj, s);
}

OpenHRP::creekGaitPatternGeneratorCpService_ptr OpenHRP::creekGaitPatternGeneratorCpService_Helper::unmarshalObjRef(cdrStream& s) {
  return ::OpenHRP::creekGaitPatternGeneratorCpService::_unmarshalObjRef(s);
}

void OpenHRP::creekGaitPatternGeneratorCpService_Helper::duplicate(::OpenHRP::creekGaitPatternGeneratorCpService_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

OpenHRP::creekGaitPatternGeneratorCpService_ptr
OpenHRP::creekGaitPatternGeneratorCpService::_duplicate(::OpenHRP::creekGaitPatternGeneratorCpService_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

OpenHRP::creekGaitPatternGeneratorCpService_ptr
OpenHRP::creekGaitPatternGeneratorCpService::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


OpenHRP::creekGaitPatternGeneratorCpService_ptr
OpenHRP::creekGaitPatternGeneratorCpService::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

OpenHRP::creekGaitPatternGeneratorCpService_ptr
OpenHRP::creekGaitPatternGeneratorCpService::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_creekGaitPatternGeneratorCpService _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_creekGaitPatternGeneratorCpService* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_creekGaitPatternGeneratorCpService;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId = "IDL:OpenHRP/creekGaitPatternGeneratorCpService:1.0";


OpenHRP::_objref_creekGaitPatternGeneratorCpService::~_objref_creekGaitPatternGeneratorCpService() {
  
}


OpenHRP::_objref_creekGaitPatternGeneratorCpService::_objref_creekGaitPatternGeneratorCpService(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
OpenHRP::_objref_creekGaitPatternGeneratorCpService::_ptrToObjRef(const char* id)
{
  if( id == ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId )
    return (::OpenHRP::creekGaitPatternGeneratorCpService_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId) )
    return (::OpenHRP::creekGaitPatternGeneratorCpService_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void_i_cdouble_i_cdouble_i_cdouble_i_cdouble
class _0RL_cd_781e0604e4fcc56e_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_781e0604e4fcc56e_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::Double arg_0;
  ::CORBA::Double arg_1;
  ::CORBA::Double arg_2;
  ::CORBA::Double arg_3;
};

void _0RL_cd_781e0604e4fcc56e_00000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;
  arg_1 >>= _n;
  arg_2 >>= _n;
  arg_3 >>= _n;

}

void _0RL_cd_781e0604e4fcc56e_00000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::Double&)arg_0 <<= _n;
  (::CORBA::Double&)arg_1 <<= _n;
  (::CORBA::Double&)arg_2 <<= _n;
  (::CORBA::Double&)arg_3 <<= _n;

}

const char* const _0RL_cd_781e0604e4fcc56e_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_781e0604e4fcc56e_10000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_781e0604e4fcc56e_00000000* tcd = (_0RL_cd_781e0604e4fcc56e_00000000*)cd;
  OpenHRP::_impl_creekGaitPatternGeneratorCpService* impl = (OpenHRP::_impl_creekGaitPatternGeneratorCpService*) svnt->_ptrToInterface(OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId);
  impl->setTargetPos(tcd->arg_0, tcd->arg_1, tcd->arg_2, tcd->arg_3);


}

void OpenHRP::_objref_creekGaitPatternGeneratorCpService::setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time)
{
  _0RL_cd_781e0604e4fcc56e_00000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_10000000, "setTargetPos", 13);
  _call_desc.arg_0 = x;
  _call_desc.arg_1 = y;
  _call_desc.arg_2 = th;
  _call_desc.arg_3 = time;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void
class _0RL_cd_781e0604e4fcc56e_20000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_781e0604e4fcc56e_20000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
    
  
  static const char* const _user_exns[];

  
};

const char* const _0RL_cd_781e0604e4fcc56e_20000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_781e0604e4fcc56e_30000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekGaitPatternGeneratorCpService* impl = (OpenHRP::_impl_creekGaitPatternGeneratorCpService*) svnt->_ptrToInterface(OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId);
  impl->startStepping();


}

void OpenHRP::_objref_creekGaitPatternGeneratorCpService::startStepping()
{
  _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_30000000, "startStepping", 14);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_781e0604e4fcc56e_40000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekGaitPatternGeneratorCpService* impl = (OpenHRP::_impl_creekGaitPatternGeneratorCpService*) svnt->_ptrToInterface(OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId);
  impl->stopStepping();


}

void OpenHRP::_objref_creekGaitPatternGeneratorCpService::stopStepping()
{
  _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_40000000, "stopStepping", 13);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_781e0604e4fcc56e_50000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekGaitPatternGeneratorCpService* impl = (OpenHRP::_impl_creekGaitPatternGeneratorCpService*) svnt->_ptrToInterface(OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId);
  impl->test();


}

void OpenHRP::_objref_creekGaitPatternGeneratorCpService::test()
{
  _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_50000000, "test", 5);


  _invoke(_call_desc);



}
OpenHRP::_pof_creekGaitPatternGeneratorCpService::~_pof_creekGaitPatternGeneratorCpService() {}


omniObjRef*
OpenHRP::_pof_creekGaitPatternGeneratorCpService::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::OpenHRP::_objref_creekGaitPatternGeneratorCpService(ior, id);
}


::CORBA::Boolean
OpenHRP::_pof_creekGaitPatternGeneratorCpService::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId) )
    return 1;
  
  return 0;
}

const OpenHRP::_pof_creekGaitPatternGeneratorCpService _the_pof_OpenHRP_mcreekGaitPatternGeneratorCpService;

OpenHRP::_impl_creekGaitPatternGeneratorCpService::~_impl_creekGaitPatternGeneratorCpService() {}


::CORBA::Boolean
OpenHRP::_impl_creekGaitPatternGeneratorCpService::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "setTargetPos") ) {

    _0RL_cd_781e0604e4fcc56e_00000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_10000000, "setTargetPos", 13, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "startStepping") ) {

    _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_30000000, "startStepping", 14, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "stopStepping") ) {

    _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_40000000, "stopStepping", 13, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "test") ) {

    _0RL_cd_781e0604e4fcc56e_20000000 _call_desc(_0RL_lcfn_781e0604e4fcc56e_50000000, "test", 5, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
OpenHRP::_impl_creekGaitPatternGeneratorCpService::_ptrToInterface(const char* id)
{
  if( id == ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId )
    return (::OpenHRP::_impl_creekGaitPatternGeneratorCpService*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId) )
    return (::OpenHRP::_impl_creekGaitPatternGeneratorCpService*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
OpenHRP::_impl_creekGaitPatternGeneratorCpService::_mostDerivedRepoId()
{
  return ::OpenHRP::creekGaitPatternGeneratorCpService::_PD_repoId;
}

POA_OpenHRP::creekGaitPatternGeneratorCpService::~creekGaitPatternGeneratorCpService() {}
