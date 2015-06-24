#ifndef ROTATIONAL_INTERPOLATOR_H
#define ROTATIONAL_INTERPOLATOR_H

#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>

#include "Interpolator.h"

namespace creek
{
  class RotationalInterpolator : public Interpolator
  {
  public:
    RotationalInterpolator(double in_dt, InterpolationType in_itype=LINEAR);

    void get(tvmet::Matrix<double, 3, 3> &out_val, bool popp=true);
    inline tvmet::Matrix<double, 3, 3> get(bool popp=true) {
      tvmet::Matrix<double, 3, 3> out;
      get(out, popp);
      return out;
    }

    bool calcInterpolation(const tvmet::Matrix<double, 3, 3> &R_cur, const tvmet::Matrix<double, 3, 3> &R_ref, const tvmet::Vector<double, 3> &axis, double in_time, double in_2_delta=0);



  private:
    //-----------------------------------------------------------------------
    // from OpenHRP
    //
    void calcRodrigues(tvmet::Matrix<double, 3, 3> &out_R, const tvmet::Vector<double, 3> &axis, double q);    
    inline tvmet::Matrix<double, 3, 3> rodrigues(const tvmet::Vector<double, 3> &axis, double q){
      tvmet::Matrix<double, 3, 3> R;
      calcRodrigues(R, axis, q);
      return R;
    }

    tvmet::Vector<double, 3> omegaFromRot(const tvmet::Matrix<double, 3, 3> &r);
    //-----------------------------------------------------------------------


    tvmet::Vector<double, 3>  axis_a, axis_b;
    double   angle_a, angle_b;
    tvmet::Matrix<double, 3, 3> R_base;
  };

};


#endif
