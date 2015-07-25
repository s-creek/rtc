// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "creekPhysicalCameraService.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



OpenHRP::creekPhysicalCameraService_ptr OpenHRP::creekPhysicalCameraService_Helper::_nil() {
  return ::OpenHRP::creekPhysicalCameraService::_nil();
}

::CORBA::Boolean OpenHRP::creekPhysicalCameraService_Helper::is_nil(::OpenHRP::creekPhysicalCameraService_ptr p) {
  return ::CORBA::is_nil(p);

}

void OpenHRP::creekPhysicalCameraService_Helper::release(::OpenHRP::creekPhysicalCameraService_ptr p) {
  ::CORBA::release(p);
}

void OpenHRP::creekPhysicalCameraService_Helper::marshalObjRef(::OpenHRP::creekPhysicalCameraService_ptr obj, cdrStream& s) {
  ::OpenHRP::creekPhysicalCameraService::_marshalObjRef(obj, s);
}

OpenHRP::creekPhysicalCameraService_ptr OpenHRP::creekPhysicalCameraService_Helper::unmarshalObjRef(cdrStream& s) {
  return ::OpenHRP::creekPhysicalCameraService::_unmarshalObjRef(s);
}

void OpenHRP::creekPhysicalCameraService_Helper::duplicate(::OpenHRP::creekPhysicalCameraService_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

OpenHRP::creekPhysicalCameraService_ptr
OpenHRP::creekPhysicalCameraService::_duplicate(::OpenHRP::creekPhysicalCameraService_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

OpenHRP::creekPhysicalCameraService_ptr
OpenHRP::creekPhysicalCameraService::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


OpenHRP::creekPhysicalCameraService_ptr
OpenHRP::creekPhysicalCameraService::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

OpenHRP::creekPhysicalCameraService_ptr
OpenHRP::creekPhysicalCameraService::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_creekPhysicalCameraService _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_creekPhysicalCameraService* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_creekPhysicalCameraService;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* OpenHRP::creekPhysicalCameraService::_PD_repoId = "IDL:OpenHRP/creekPhysicalCameraService:1.0";


OpenHRP::_objref_creekPhysicalCameraService::~_objref_creekPhysicalCameraService() {
  
}


OpenHRP::_objref_creekPhysicalCameraService::_objref_creekPhysicalCameraService(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::OpenHRP::creekPhysicalCameraService::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
OpenHRP::_objref_creekPhysicalCameraService::_ptrToObjRef(const char* id)
{
  if( id == ::OpenHRP::creekPhysicalCameraService::_PD_repoId )
    return (::OpenHRP::creekPhysicalCameraService_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::OpenHRP::creekPhysicalCameraService::_PD_repoId) )
    return (::OpenHRP::creekPhysicalCameraService_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void
class _0RL_cd_c3fffc2852ea9db9_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_c3fffc2852ea9db9_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
    
  
  static const char* const _user_exns[];

  
};

const char* const _0RL_cd_c3fffc2852ea9db9_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_c3fffc2852ea9db9_10000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekPhysicalCameraService* impl = (OpenHRP::_impl_creekPhysicalCameraService*) svnt->_ptrToInterface(OpenHRP::creekPhysicalCameraService::_PD_repoId);
  impl->window();


}

void OpenHRP::_objref_creekPhysicalCameraService::window()
{
  _0RL_cd_c3fffc2852ea9db9_00000000 _call_desc(_0RL_lcfn_c3fffc2852ea9db9_10000000, "window", 7);


  _invoke(_call_desc);



}
OpenHRP::_pof_creekPhysicalCameraService::~_pof_creekPhysicalCameraService() {}


omniObjRef*
OpenHRP::_pof_creekPhysicalCameraService::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::OpenHRP::_objref_creekPhysicalCameraService(ior, id);
}


::CORBA::Boolean
OpenHRP::_pof_creekPhysicalCameraService::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::OpenHRP::creekPhysicalCameraService::_PD_repoId) )
    return 1;
  
  return 0;
}

const OpenHRP::_pof_creekPhysicalCameraService _the_pof_OpenHRP_mcreekPhysicalCameraService;

OpenHRP::_impl_creekPhysicalCameraService::~_impl_creekPhysicalCameraService() {}


::CORBA::Boolean
OpenHRP::_impl_creekPhysicalCameraService::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "window") ) {

    _0RL_cd_c3fffc2852ea9db9_00000000 _call_desc(_0RL_lcfn_c3fffc2852ea9db9_10000000, "window", 7, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
OpenHRP::_impl_creekPhysicalCameraService::_ptrToInterface(const char* id)
{
  if( id == ::OpenHRP::creekPhysicalCameraService::_PD_repoId )
    return (::OpenHRP::_impl_creekPhysicalCameraService*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::OpenHRP::creekPhysicalCameraService::_PD_repoId) )
    return (::OpenHRP::_impl_creekPhysicalCameraService*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
OpenHRP::_impl_creekPhysicalCameraService::_mostDerivedRepoId()
{
  return ::OpenHRP::creekPhysicalCameraService::_PD_repoId;
}

POA_OpenHRP::creekPhysicalCameraService::~creekPhysicalCameraService() {}
