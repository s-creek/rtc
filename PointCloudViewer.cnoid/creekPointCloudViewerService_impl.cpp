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

void creekPointCloudViewerService_impl::test()
{
  if( m_comp != NULL )
    m_comp->test();
}
