#include "Interpolator.h"
#include "RotationalInterpolator.h"
typedef tvmet::Matrix<double, 3, 3> Matrix33;
typedef tvmet::Vector<double, 3> Vector3;

#include <iostream>
#include <stdio.h>
#include <fstream>

int main()
{
  unsigned int dim(3);
  double dt(0.05);

  creek::Interpolator *interpolators[4];
  interpolators[0] = new creek::Interpolator(dim, dt, creek::LINEAR);
  interpolators[1] = new creek::Interpolator(dim, dt, creek::CUBIC);
  interpolators[2] = new creek::Interpolator(dim, dt, creek::QUINTIC);
  interpolators[3] = new creek::Interpolator(dim, dt, creek::QUARTIC_LINEAR);


  double xs[] = { 0.0,  0.0,  0.0};  double dxs[] = { 0.2, 0.0, 0.0};  double ddxs[] = { 0.0, 0.0, 0.0};
  double xf[] = { 5.0, -5.0, 10.0};  double dxf[] = { 0.0, 0.0, 0.0};  double ddxf[] = { 0.0, 0.0, 0.0};


  bool success = true;
  for(int i = 0; i < 4; i++) {
    if( !interpolators[i]->calcInterpolation(&xs[0], &dxs[0], &ddxs[0], &xf[0], &dxf[0], &ddxf[0], 10.0, 2.5) ) {
      std::cerr << "error : [calcInterpolation] interpolation type = " << interpolators[i]->interpolationType() << std::endl;
      success = false;
    }
  }
  if(!success)
    return 0;


  std::ofstream log("log.dat");
  double out[dim];
  while( !interpolators[0]->empty() ) {

    for(int i = 0; i < 4; i++) {
      interpolators[i]->get(&out[0]);
      for(unsigned int i = 0; i < interpolators[i]->dimension(); i++) {
	printf("  %8.4lf", out[i]);
      }
      log << " " << out[0];
      std::cout << "   |";
    }
    std::cout << std::endl;
    log << std::endl;
  }
  log.close();



  creek::RotationalInterpolator *rot_inter = new creek::RotationalInterpolator(dt, creek::QUINTIC);
  Matrix33 Rs( tvmet::identity<Matrix33>() );
  Matrix33 Rf( 0 );  Rf(0,1) = -1.0;  Rf(1,0) = 1.0;  Rf(2,2) = 1.0;
  Vector3  axis(1,0,0);

  if( !rot_inter->calcInterpolation(Rs, Rf, axis, 5.0) )
    return 0;


  printf("\n\n\n\n");
  Matrix33 R;
  while( !rot_inter->empty() ) {
    rot_inter->get(R);
    std::cout << R << std::endl;
  }
}
