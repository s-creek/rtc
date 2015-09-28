// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "creekRobotStateService.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



OpenHRP::creekRobotStateService_ptr OpenHRP::creekRobotStateService_Helper::_nil() {
  return ::OpenHRP::creekRobotStateService::_nil();
}

::CORBA::Boolean OpenHRP::creekRobotStateService_Helper::is_nil(::OpenHRP::creekRobotStateService_ptr p) {
  return ::CORBA::is_nil(p);

}

void OpenHRP::creekRobotStateService_Helper::release(::OpenHRP::creekRobotStateService_ptr p) {
  ::CORBA::release(p);
}

void OpenHRP::creekRobotStateService_Helper::marshalObjRef(::OpenHRP::creekRobotStateService_ptr obj, cdrStream& s) {
  ::OpenHRP::creekRobotStateService::_marshalObjRef(obj, s);
}

OpenHRP::creekRobotStateService_ptr OpenHRP::creekRobotStateService_Helper::unmarshalObjRef(cdrStream& s) {
  return ::OpenHRP::creekRobotStateService::_unmarshalObjRef(s);
}

void OpenHRP::creekRobotStateService_Helper::duplicate(::OpenHRP::creekRobotStateService_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

OpenHRP::creekRobotStateService_ptr
OpenHRP::creekRobotStateService::_duplicate(::OpenHRP::creekRobotStateService_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

OpenHRP::creekRobotStateService_ptr
OpenHRP::creekRobotStateService::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


OpenHRP::creekRobotStateService_ptr
OpenHRP::creekRobotStateService::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

OpenHRP::creekRobotStateService_ptr
OpenHRP::creekRobotStateService::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_creekRobotStateService _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_creekRobotStateService* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_creekRobotStateService;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* OpenHRP::creekRobotStateService::_PD_repoId = "IDL:OpenHRP/creekRobotStateService:1.0";


OpenHRP::_objref_creekRobotStateService::~_objref_creekRobotStateService() {
  
}


OpenHRP::_objref_creekRobotStateService::_objref_creekRobotStateService(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::OpenHRP::creekRobotStateService::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
OpenHRP::_objref_creekRobotStateService::_ptrToObjRef(const char* id)
{
  if( id == ::OpenHRP::creekRobotStateService::_PD_repoId )
    return (::OpenHRP::creekRobotStateService_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::OpenHRP::creekRobotStateService::_PD_repoId) )
    return (::OpenHRP::creekRobotStateService_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void
class _0RL_cd_2b7c892ca66199d1_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_2b7c892ca66199d1_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
    
  
  static const char* const _user_exns[];

  
};

const char* const _0RL_cd_2b7c892ca66199d1_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_2b7c892ca66199d1_10000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekRobotStateService* impl = (OpenHRP::_impl_creekRobotStateService*) svnt->_ptrToInterface(OpenHRP::creekRobotStateService::_PD_repoId);
  impl->test();


}

void OpenHRP::_objref_creekRobotStateService::test()
{
  _0RL_cd_2b7c892ca66199d1_00000000 _call_desc(_0RL_lcfn_2b7c892ca66199d1_10000000, "test", 5);


  _invoke(_call_desc);



}
OpenHRP::_pof_creekRobotStateService::~_pof_creekRobotStateService() {}


omniObjRef*
OpenHRP::_pof_creekRobotStateService::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::OpenHRP::_objref_creekRobotStateService(ior, id);
}


::CORBA::Boolean
OpenHRP::_pof_creekRobotStateService::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::OpenHRP::creekRobotStateService::_PD_repoId) )
    return 1;
  
  return 0;
}

const OpenHRP::_pof_creekRobotStateService _the_pof_OpenHRP_mcreekRobotStateService;

OpenHRP::_impl_creekRobotStateService::~_impl_creekRobotStateService() {}


::CORBA::Boolean
OpenHRP::_impl_creekRobotStateService::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "test") ) {

    _0RL_cd_2b7c892ca66199d1_00000000 _call_desc(_0RL_lcfn_2b7c892ca66199d1_10000000, "test", 5, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
OpenHRP::_impl_creekRobotStateService::_ptrToInterface(const char* id)
{
  if( id == ::OpenHRP::creekRobotStateService::_PD_repoId )
    return (::OpenHRP::_impl_creekRobotStateService*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::OpenHRP::creekRobotStateService::_PD_repoId) )
    return (::OpenHRP::_impl_creekRobotStateService*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
OpenHRP::_impl_creekRobotStateService::_mostDerivedRepoId()
{
  return ::OpenHRP::creekRobotStateService::_PD_repoId;
}

POA_OpenHRP::creekRobotStateService::~creekRobotStateService() {}
