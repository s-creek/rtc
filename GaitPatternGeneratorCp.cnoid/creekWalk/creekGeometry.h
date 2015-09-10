// -*- c++ -*-

#ifndef CREEK_GEOMETRY_H
#define CREEK_GEOMETRY_H

#include <creekTypesEigen.h>

namespace creek
{
  bool getPlaneEquation(const Vector3 &in_a, const Vector3 &in_b, const Vector3 &in_c, Vector4 &out);
  bool getPlaneEquation(const Vector3 &in_p, const Vector3 &in_n, Vector4 &out);
  bool getIntersectPlaneAndLine(const Vector4 &in_plane, const Vector3 &in_A, const Vector3 &in_B, Vector3 &out);
  Matrix3 midYaw(const Matrix3 &in_a, const Matrix3 &in_b);
}

#endif
