#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <iostream>
#include <deque>

namespace creek 
{
  enum InterpolationType {
    LINEAR,
    CUBIC,
    QUINTIC,
    QUARTIC_LINEAR
  };


  class Interpolator
  {
  public:
    Interpolator(unsigned int in_dim, double in_dt, InterpolationType in_itype=LINEAR);
    ~Interpolator();

    bool setInterpolationType (InterpolationType in_itype);
    void clear(); 
    void get(double* out, bool popp=true);
    void pop();

    inline unsigned int dimension() {
      return m_dim;
    }

    inline InterpolationType interpolationType() {
      return m_itype;
    }
    
    inline bool empty() {
      return m_seq.empty();
    }
  
    inline unsigned int numSequence() {
      return m_seq.size();
    }

    /*
    inline std::deque<double*> *sequence() {
      return &m_seq;
    }
    */

    inline double* front() {
      return m_seq.front();
    }
    
    inline double* back() {
      return m_seq.back();
    }

    inline bool calcInterpolation(const double *in_xs, const double *in_xf, double in_time, double in_2_delta=0) {
      return calcInterpolation(in_xs, 0, 0, in_xf, 0, 0, in_time, in_2_delta);
    }
    inline bool calcInterpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time, double in_2_delta=0) {
      return calcInterpolation(in_xs, in_dxs, 0, in_xf, in_dxf, 0, in_time, in_2_delta);
    }
    bool calcInterpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, 
			   const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time, double in_2_delta=0);

    Interpolator& operator=(const Interpolator& org);


  private:
    unsigned int m_dim;
    double m_dt;
    long unsigned int m_memsize;
    InterpolationType m_itype;

    std::deque<double*> m_seq;

    double *m_last_out;

    double *m_dxs, *m_dxf;
    double *m_ddxs, *m_ddxf;

    double m_eps;
    bool linear_interpolation(const double *in_xs, const double *in_xf, double in_time);
    bool cubic_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time);
    bool quintic_interpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time);
    bool qlq_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time, double in_2_delta);
  };
};

#endif
