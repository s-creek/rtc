// -*-C++-*-

#include "creekCameraViewerService.hh"

#ifndef CREEKCAMERAVIEWERSERVICE_IMPL_H
#define CREEKCAMERAVIEWERSERVICE_IMPL_H
 
class creekCameraViewerService_impl
  : public virtual POA_OpenHRP::creekCameraViewerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekCameraViewerService_impl();
  virtual ~creekCameraViewerService_impl();

  void setDraw();
  inline bool draw() { return m_draw; }

private:
  bool m_draw;
};


#endif // CREEKCAMERAVIEWERSERVICE_IMPL_H


