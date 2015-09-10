#include <iostream>
#include <fstream>

#include <WalkPlanner.h>
#include <RobotCnoid.h>
#include <StepSequence.h>
#include <creekGeometry.h>
#include <cnoid/BodyLoader>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkTransform.h>

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
  //robot->rootLink()->p() << 0.0, 0.0, 0.854;
  //robot->setSoleOffset(cnoid::Vector3(0.0, 0.0, -0.1), cnoid::Vector3(0.0, 0.0, -0.1));
  robot->rootLink()->p() << 0.0, 0.0, 0.846;
  robot->setSoleOffset(cnoid::Vector3(0.0, 0.0, 0.0), cnoid::Vector3(0.0, 0.0, 0.0));
  robot->calcForwardKinematics();



  //
  // walk planner
  //
  WalkPlanner wp;
  wp.setRobot(robot);
  wp.setTime(0.005, 0.7, 0.1);
  wp.setFootSize(0.160, 0.100, 0.055, 0.055);
  wp.setOffset(0.0, -0.01);

  creek::Vector3 zmp;
  zmp = ( robot->rfoot()->p() + robot->lfoot()->p() ) / 2.0;
  wp.initParameters(zmp);
  wp.setStepping(true);

  creek::Position rfoot = robot->rfoot()->position();
  creek::Position lfoot = robot->lfoot()->position();

  wp.addStep(rfoot, creek::RFOOT);
  wp.addStep(lfoot, creek::LFOOT);
  wp.addStep(rfoot, creek::RFOOT);
  wp.addStep(lfoot, creek::LFOOT);



  // init pcl
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters();
  viewer->setSize(1280, 720);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(0.1);
  viewer->setCameraPosition(-10, 0, 3,
			    0, 0, 1.5,
			    0, 0, 1);

  vtkSmartPointer<vtkTransform> traR = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform> traL = vtkSmartPointer<vtkTransform>::New();
  viewer->addModelFromPLYFile("../RLEG_LINK5.ply", traR, "rfoot");
  viewer->addModelFromPLYFile("../LLEG_LINK5.ply", traL, "lfoot");

  pcl::PointXYZRGB com(255,0,0);
  com.x = wp.step().front().com[0];
  com.y = wp.step().front().com[1];
  com.z = wp.step().front().com[2];
  viewer->addSphere(com, 0.025, "com");

  pcl::PointXYZRGB cp(0,10,255);
  cp.x = wp.step().front().cp[0];
  cp.y = wp.step().front().cp[1];
  cp.z = wp.step().front().cp[2];
  viewer->addSphere(cp, 0.025, "cp");



  //
  // view
  //
  double dt(0.005);
  int index=0;
  while ( !viewer->wasStopped() ) { // Display the visualiser until 'q' key is pressed

    {
      robot->rfoot()->position() = wp.step().at(index).rfoot;

      creek::Vector3    pos(robot->rfoot()->p());
      Eigen::AngleAxisd aa(robot->rfoot()->R());
      traR->Identity();
      traR->Translate(pos[0], pos[1], pos[2]);
      traR->RotateWXYZ(aa.angle()/M_PI*180.0, aa.axis()[0], aa.axis()[1], aa.axis()[2]);
    }
    {
      robot->lfoot()->position() = wp.step().at(index).lfoot;

      creek::Vector3    pos(robot->lfoot()->p());
      Eigen::AngleAxisd aa(robot->lfoot()->R());
      traL->Identity();
      traL->Translate(pos[0], pos[1], pos[2]);
      traL->RotateWXYZ(aa.angle()/M_PI*180.0, aa.axis()[0], aa.axis()[1], aa.axis()[2]);
    }
    {
      com.x = wp.step().at(index).com[0];
      com.y = wp.step().at(index).com[1];
      com.z = wp.step().at(index).com[2];
      viewer->updateSphere(com, 0.025, 1.0, 0.0, 0.0, "com");
    }
    {
      cp.x = wp.step().at(index).cp[0];
      cp.y = wp.step().at(index).cp[1];
      cp.z = wp.step().at(index).cp[2];
      viewer->updateSphere(cp, 0.025, 0.0, 0.1, 1.0, "cp");
    }


    index++;
    if( index >= wp.step().size() )
      index = 0;

    viewer->spinOnce(dt*1000);
    boost::this_thread::sleep(boost::posix_time::microseconds(dt*1000000));
  }


  return 0;
}
