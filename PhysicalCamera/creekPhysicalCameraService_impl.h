// -*-C++-*-

#include "creekPhysicalCameraService.hh"

#ifndef CREEK_PHYSICAL_CAMERA_SERVICE_IMPL_H
#define CREEK_PHYSICAL_CAMERA_SERVICE_IMPL_H

class creekPhysicalCamera;

class creekPhysicalCameraService_impl
  : public virtual POA_OpenHRP::creekPhysicalCameraService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekPhysicalCameraService_impl();
  virtual ~creekPhysicalCameraService_impl();
  
  void window();

  void setComponent (creekPhysicalCamera * i_comp) {
    m_comp = i_comp;
  }

private:
  creekPhysicalCamera *m_comp;
};

#endif // CREEK_PHYSICAL_CAMERA_SERVICE_IMPL_H


