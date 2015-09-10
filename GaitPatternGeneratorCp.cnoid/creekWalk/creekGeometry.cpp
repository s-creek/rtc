#include "creekGeometry.h"

namespace creek
{
  bool getPlaneEquation(const Vector3 &in_a, const Vector3 &in_b, const Vector3 &in_c, Vector4 &out)
  {
    Vector3 ba(in_b-in_a);
    Vector3 ca(in_c-in_a);

    Vector3 n = ba.cross(ca);
    if( n.norm() < 1.0e-12 )
      return false;
    if( n[2] < 0)
      n = -n;
 
    n.normalize();
    double d = -in_a.dot(n);

    out[0] = n[0];
    out[1] = n[1];
    out[2] = n[2];
    out[3] = d;

    return true;
  }


  bool getPlaneEquation(const Eigen::Vector3d &in_p, const Eigen::Vector3d &in_n, Eigen::Vector4d &out)
  {
    Vector3 n(in_n);
    if( n.norm() < 1.0e-12 )
      return false;
    if( n[2] < 0)
      n = -n;

    n.normalize();
    double d = -in_p.dot(n);

    out[0] = n[0];
    out[1] = n[1];
    out[2] = n[2];
    out[3] = d;

    return true;
  }


  // ref : http://www.hiramine.com/programming/graphics/3d_planesegmentintersection.html
  bool getIntersectPlaneAndLine(const Vector4 &in_plane, const Vector3 &in_A, const Vector3 &in_B, Vector3 &out)
  {
    Vector3 n(in_plane[0], in_plane[1], in_plane[2]);
    double  d = in_plane[3];
    Vector3 ab(in_B-in_A);

    if( std::fabs(n.dot(ab)) < 1.0e-12 )
      return false;

    double t = (-d - n.dot(in_A)) / n.dot(ab);
    out = in_A + t * ab;

    return true;
  }


  Matrix3 midYaw(const Matrix3 &in_a, const Matrix3 &in_b)
  {
    Vector3 rpya = in_a.eulerAngles(2,1,0);
    Vector3 rpyb = in_b.eulerAngles(2,1,0);

    double dyaw = rpyb[0] - rpya[0];
    while( dyaw > M_PI || dyaw < -M_PI ) {
      if( dyaw > M_PI )
	dyaw -= (2*M_PI);
      else if( dyaw < -M_PI )
	dyaw += (2*M_PI);
    }
    double yaw = rpya[0] + dyaw/2.0;
    return Eigen::AngleAxisd(yaw, Vector3::UnitZ()).toRotationMatrix();
  }
}
