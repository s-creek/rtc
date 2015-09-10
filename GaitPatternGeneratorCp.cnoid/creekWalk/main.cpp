#include <iostream>
#include <fstream>
#include <WalkPlanner.h>
#include <RobotCnoid.h>
#include <StepSequence.h>
#include <creekGeometry.h>
#include <cnoid/BodyLoader>

int main()
{
  //
  // init robot
  //
  RobotPtr robot = new Robot;
  cnoid::BodyLoader bl;
  if( !bl.load( robot, "/home/ogawa/workspace/cnoid/model/JVRC-1/main.wrl" ) ) {
    std::cerr << "model load error" << std::endl;
    return 0;
  }
  if( !robot->setJointPath("R_ANKLE_P", "PELVIS", "L_ANKLE_P", "PELVIS") )
    return 0;
  robot->rootLink()->p() << 0.0, 0.0, 0.854;
  robot->setSoleOffset(cnoid::Vector3(0.0, 0.0, -0.108), cnoid::Vector3(0.0, 0.0, -0.108));
  robot->calcForwardKinematics();


  //
  // walk planner
  //
  WalkPlanner wp;
  wp.setRobot(robot);
  wp.setTime(0.005, 0.7, 0.1);
  wp.setFootSize(0.160, 0.100, 0.055, 0.055);
  wp.setOffset(0.0, -0.01);


  cnoid::Vector3 rzmp, lzmp, zmp;
  rzmp = robot->rfoot()->p();
  lzmp = robot->lfoot()->p();
  zmp  = (rzmp+lzmp) / 2.0;

  std::cout << wp.getFootType(rzmp) << std::endl;
  std::cout << wp.getFootType(lzmp) << std::endl;
  std::cout << wp.getFootType(zmp, 0.01) << std::endl;


  wp.test();
  robot->link("R_HIP_P")->q() = -0.5;
  robot->link("R_KNEE")->q() = 1.0;
  robot->link("R_ANKLE_P")->q() = -0.5;
  robot->calcForwardKinematics();
  wp.test();


  cnoid::Vector3 refP(0.02,  -0.096,  0.15);
  cnoid::Matrix3 refR(cnoid::Matrix3::Identity());
  if( !robot->rleg()->calcInverseKinematics(refP, refR) )
    std::cout << "error" << std::endl;
  wp.test();
  

  std::cout << std::endl;
  for(int i=0; i<robot->rleg()->numJoints(); i++) {
    cnoid::Link *link = robot->rleg()->joint(i);
    std::cout << link->name() << " : " << link->p().format(creek::IOvec()) << " [m]" << std::endl;
    //std::cout << link->R().format(creek::IOmat()) << std::endl;
  }

  //zmp << 0.02426, -0.1164, 0.0;
  zmp << 0.185, 0.0, 0.0;
  std::cout << wp.getFootType(zmp, 0.01) << std::endl;


  cnoid::Vector3 A(1,0,0), B(0,1,0), C(0,0,1);
  cnoid::Vector4 plane;
  if( creek::getPlaneEquation(A, B, C, plane) )
    std::cout << plane.format(creek::IOvec()) << std::endl;


  cnoid::Vector3 D(0,0,0), E(1,1,1), p;
  if( creek::getIntersectPlaneAndLine(plane, D, E, p) )
    std::cout << p.format(creek::IOvec()) << std::endl;





  // for(int i=0; i<robot->numJoints(); i++) {
  //   robot->joint(i)->q() = 0.0;
  // }
  // robot->calcForwardKinematics();

  
  wp.setStepping();
  wp.initParameters(zmp);
  std::cout << "com = " << wp.step().sequence().front().com.format(creek::IOvec()) << std::endl;
  if( wp.addStep(robot->rfoot()->position(), creek::RFOOT, true) ) {
    std::cout << wp.step().sequence().size() << std::endl;
    std::cout << wp.step().sequence().back().zmp.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().com.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().cp.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().rfoot.translation().format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().lfoot.translation().format(creek::IOvec()) << std::endl;
  }
  else {
    std::cout << "add step error" << std::endl;
  }

  creek::Position pos = robot->lfoot()->position();
  pos.translation()[0] += 0.1;
  if( wp.addStep(pos, creek::LFOOT) ) {
    std::cout << wp.step().sequence().size() << std::endl;
    std::cout << wp.step().sequence().back().zmp.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().com.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().cp.format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().rfoot.translation().format(creek::IOvec()) << std::endl;
    std::cout << wp.step().sequence().back().lfoot.translation().format(creek::IOvec()) << std::endl;
   }
  else {
    std::cout << "add step error" << std::endl;
  }


  std::ofstream log("log.dat");
  for(int i=0; i<wp.step().sequence().size(); i++) {
    log << "  " << wp.step().sequence().at(i).zmp[1]
	<< "  " << wp.step().sequence().at(i).com[1]
	<< "  " << wp.step().sequence().at(i).cp[1]
	<< "  " << wp.step().sequence().at(i).rfoot.translation()[2]
	<< "  " << wp.step().sequence().at(i).lfoot.translation()[2]
	<< std::endl;
  }
  log.close();

  return 0;
}
