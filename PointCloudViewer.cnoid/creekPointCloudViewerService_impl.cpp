// -*-C++-*-

#include "creekPointCloudViewerService_impl.h"
#include "creekPointCloudViewer.h"

creekPointCloudViewerService_impl::creekPointCloudViewerService_impl()
  : m_comp(NULL)
{
}

creekPointCloudViewerService_impl::~creekPointCloudViewerService_impl()
{
}

void creekPointCloudViewerService_impl::start()
{
  if( m_comp != NULL )
    m_comp->start();
}

void creekPointCloudViewerService_impl::stop()
{
  if( m_comp != NULL )
    m_comp->stop();
}


bool creekPointCloudViewerService_impl::detectLandingPoint(double x, double y, double w, int ft)
{
  if( m_comp != NULL )
    return m_comp->detectLandingPoint(x,y,w,ft);
  else
    return false;
}

void creekPointCloudViewerService_impl::test()
{
  if( m_comp != NULL )
    m_comp->test();
}
