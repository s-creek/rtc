#include "RotationalInterpolator.h"

using namespace creek;

typedef tvmet::Matrix<double, 3, 3> Matrix33;
typedef tvmet::Vector<double, 3> Vector3;


RotationalInterpolator::RotationalInterpolator(double in_dt, InterpolationType in_itype) :
  Interpolator(1, in_dt, in_itype)
{
  axis_a  = axis_b  = Vector3(1,0,0);
  angle_a = angle_b = 0.0;
  R_base  = tvmet::identity<Matrix33>();
}


void RotationalInterpolator::get(Matrix33 &out_val, bool popp)
{
  double   variation;
  Interpolator::get(&variation, popp);


  double   angle_a_tmp, angle_b_tmp;
  Matrix33 R_a_tmp, R_b_tmp;

  angle_a_tmp = variation * angle_a;
  angle_b_tmp = variation * angle_b;
  
  R_a_tmp = rodrigues(axis_a, angle_a_tmp);
  R_b_tmp = rodrigues(axis_b, angle_b_tmp);
  out_val = R_base * R_a_tmp * R_b_tmp;
}


bool RotationalInterpolator::calcInterpolation(const Matrix33& R_cur, const Matrix33& R_ref, const Vector3& axis, double in_time, double in_2_delta)
{
  double xs = 0, xf = 1;
  if( !Interpolator::calcInterpolation(&xs, &xf, in_time, in_2_delta) )
    return false;


  // 誤差回転行列Rの導出
  Matrix33 dR(tvmet::trans(R_cur) * R_ref);


  // target方向への回転の導出
  double cos_angle_a = tvmet::dot(axis, (dR * axis));
  if( cos_angle_a > (1.0 - 1.0e-6) )
    angle_a = 0;
  else if(cos_angle_a < -(1.0 - 1.0e-6) )
    angle_a = 3.1415;
  else
    angle_a = acos(cos_angle_a);


  if(angle_a > 1e-3){
    axis_a = tvmet::cross(axis, (dR * axis));
    axis_a /= tvmet::norm2(axis_a);
  }
  else{
    axis_a = Vector3(1,0,0);
  }


  Matrix33 R_a(rodrigues(axis_a, angle_a));


  // target方向以外への回転の導出
  Matrix33 R_b(tvmet::trans(R_a) * dR);
  Vector3  omega_b(omegaFromRot(R_b));
  

  angle_b = tvmet::norm2(omega_b);


  if(fabs(angle_b) > 1.0e-6) {
    axis_b  = omega_b / angle_b;
  }
  else {
    axis_b = Vector3(0,1,0);
  }


  R_base = R_cur;

  //std::cout  << "\n\n" << angle_a << "\n\n" << angle_b << "\n\n" << axis_a << "\n\n" << axis_b << "\n\n" << R_base << std::endl;

  return true;
}


void RotationalInterpolator::calcRodrigues(Matrix33& out_R, const Vector3& axis, double q)
{
  // E + a_hat*sin(q) + a_hat*a_hat*(1-cos(q))
  //
  //    |  0 -az  ay|
  // =E+| az   0 -ax|*s + a_hat*a_hat*v
  //    |-ay  ax   0|
  //
  //    |  0 -az  ay|     |-az*az-ay*ay        ax*ay        az*ax|
  // =E+| az   0 -ax|*s + |       ax*ay -az*az-ax*ax        ay*az|*v
  //    |-ay  ax   0|     |       az*ax        ay*az -ax*ax-ay*ay|
  //
  //  |1-az*az*v-ay*ay*v     -az*s+ax*ay*v      ay*s+az*ax*v|
  // =|     az*s+ax*ay*v 1-az*az*v-ax*ax*v     -ax*s+ay+az*v|
  //  |    -ay*s+az*ax*v      ax*s+ay*az*v 1-ax*ax*v-ay*ay*v|
  //
  
  const double sth = sin(q);
  const double vth = 1.0 - cos(q);
  
  double ax = axis(0);
  double ay = axis(1);
  double az = axis(2);
  
  const double axx = ax*ax*vth;
  const double ayy = ay*ay*vth;
  const double azz = az*az*vth;
  const double axy = ax*ay*vth;
  const double ayz = ay*az*vth;
  const double azx = az*ax*vth;
  
  ax *= sth;
  ay *= sth;
  az *= sth;
  
  out_R = 1.0 - azz - ayy, -az + axy,       ay + azx,
    az + axy,        1.0 - azz - axx, -ax + ayz,
    -ay + azx,       ax + ayz,        1.0 - ayy - axx;
}


Vector3 RotationalInterpolator::omegaFromRot(const Matrix33& r)
{
  using ::std::numeric_limits;
  
  double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;
  
  if(fabs(alpha - 1.0) < 1.0e-6) {
    return Vector3(0.0);
    
  } else {
    double th = acos(alpha);
    double s = sin(th);
    
    if (s < numeric_limits<double>::epsilon()) {
      return Vector3(0.0);
    }
    
    double k = -0.5 * th / s;
    
    return Vector3( (r(1,2) - r(2,1)) * k,
		    (r(2,0) - r(0,2)) * k,
		    (r(0,1) - r(1,0)) * k );
  }
}
