// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "creekQrCodeReaderService.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



OpenHRP::creekQrCodeReaderService_ptr OpenHRP::creekQrCodeReaderService_Helper::_nil() {
  return ::OpenHRP::creekQrCodeReaderService::_nil();
}

::CORBA::Boolean OpenHRP::creekQrCodeReaderService_Helper::is_nil(::OpenHRP::creekQrCodeReaderService_ptr p) {
  return ::CORBA::is_nil(p);

}

void OpenHRP::creekQrCodeReaderService_Helper::release(::OpenHRP::creekQrCodeReaderService_ptr p) {
  ::CORBA::release(p);
}

void OpenHRP::creekQrCodeReaderService_Helper::marshalObjRef(::OpenHRP::creekQrCodeReaderService_ptr obj, cdrStream& s) {
  ::OpenHRP::creekQrCodeReaderService::_marshalObjRef(obj, s);
}

OpenHRP::creekQrCodeReaderService_ptr OpenHRP::creekQrCodeReaderService_Helper::unmarshalObjRef(cdrStream& s) {
  return ::OpenHRP::creekQrCodeReaderService::_unmarshalObjRef(s);
}

void OpenHRP::creekQrCodeReaderService_Helper::duplicate(::OpenHRP::creekQrCodeReaderService_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

OpenHRP::creekQrCodeReaderService_ptr
OpenHRP::creekQrCodeReaderService::_duplicate(::OpenHRP::creekQrCodeReaderService_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

OpenHRP::creekQrCodeReaderService_ptr
OpenHRP::creekQrCodeReaderService::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


OpenHRP::creekQrCodeReaderService_ptr
OpenHRP::creekQrCodeReaderService::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

OpenHRP::creekQrCodeReaderService_ptr
OpenHRP::creekQrCodeReaderService::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_creekQrCodeReaderService _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_creekQrCodeReaderService* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_creekQrCodeReaderService;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* OpenHRP::creekQrCodeReaderService::_PD_repoId = "IDL:OpenHRP/creekQrCodeReaderService:1.0";


OpenHRP::_objref_creekQrCodeReaderService::~_objref_creekQrCodeReaderService() {
  
}


OpenHRP::_objref_creekQrCodeReaderService::_objref_creekQrCodeReaderService(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::OpenHRP::creekQrCodeReaderService::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
OpenHRP::_objref_creekQrCodeReaderService::_ptrToObjRef(const char* id)
{
  if( id == ::OpenHRP::creekQrCodeReaderService::_PD_repoId )
    return (::OpenHRP::creekQrCodeReaderService_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::OpenHRP::creekQrCodeReaderService::_PD_repoId) )
    return (::OpenHRP::creekQrCodeReaderService_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void
class _0RL_cd_cd7a18f3159ad6bf_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_cd7a18f3159ad6bf_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
    
  
  static const char* const _user_exns[];

  
};

const char* const _0RL_cd_cd7a18f3159ad6bf_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_cd7a18f3159ad6bf_10000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_creekQrCodeReaderService* impl = (OpenHRP::_impl_creekQrCodeReaderService*) svnt->_ptrToInterface(OpenHRP::creekQrCodeReaderService::_PD_repoId);
  impl->test();


}

void OpenHRP::_objref_creekQrCodeReaderService::test()
{
  _0RL_cd_cd7a18f3159ad6bf_00000000 _call_desc(_0RL_lcfn_cd7a18f3159ad6bf_10000000, "test", 5);


  _invoke(_call_desc);



}
OpenHRP::_pof_creekQrCodeReaderService::~_pof_creekQrCodeReaderService() {}


omniObjRef*
OpenHRP::_pof_creekQrCodeReaderService::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::OpenHRP::_objref_creekQrCodeReaderService(ior, id);
}


::CORBA::Boolean
OpenHRP::_pof_creekQrCodeReaderService::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::OpenHRP::creekQrCodeReaderService::_PD_repoId) )
    return 1;
  
  return 0;
}

const OpenHRP::_pof_creekQrCodeReaderService _the_pof_OpenHRP_mcreekQrCodeReaderService;

OpenHRP::_impl_creekQrCodeReaderService::~_impl_creekQrCodeReaderService() {}


::CORBA::Boolean
OpenHRP::_impl_creekQrCodeReaderService::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "test") ) {

    _0RL_cd_cd7a18f3159ad6bf_00000000 _call_desc(_0RL_lcfn_cd7a18f3159ad6bf_10000000, "test", 5, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
OpenHRP::_impl_creekQrCodeReaderService::_ptrToInterface(const char* id)
{
  if( id == ::OpenHRP::creekQrCodeReaderService::_PD_repoId )
    return (::OpenHRP::_impl_creekQrCodeReaderService*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::OpenHRP::creekQrCodeReaderService::_PD_repoId) )
    return (::OpenHRP::_impl_creekQrCodeReaderService*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
OpenHRP::_impl_creekQrCodeReaderService::_mostDerivedRepoId()
{
  return ::OpenHRP::creekQrCodeReaderService::_PD_repoId;
}

POA_OpenHRP::creekQrCodeReaderService::~creekQrCodeReaderService() {}

