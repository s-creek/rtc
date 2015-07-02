#ifndef CHECK_COUNTER

namespace cc
{
  int m_stepCounter(0);
  int m_nStep(0);
}

#define CHECK_COUNTER(i)			\
  {						\
    (i)++;					\
    (i)=(i)%cc::m_nStep;			\
    if( (i) != 0 )				\
      return RTC::RTC_OK;			\
  }

#define SET_CHECK_COUNTER					\
  {								\
    double cc_dt, cc_pd_dt;					\
    coil::stringTo(cc_dt, prop["dt"].c_str());			\
    coil::stringTo(cc_pd_dt, prop["pdservo.dt"].c_str());	\
    cc::m_nStep = int( cc_dt/cc_pd_dt + 1e-12);			\
  }

#endif
