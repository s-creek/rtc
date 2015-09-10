// -*- c++ -*-

#ifndef WALK_PLANNER_H
#define WALK_PLANNER_H

#include <creekTypesEigen.h>
#include <RobotCnoid.h>
#include <StepSequence.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

class WalkPlanner
{
public:
  WalkPlanner();

  inline void setRobot(RobotPtr in_robot) {
    m_robot = in_robot;
  }
  void setTime(double in_dt, double in_timeSwg, double in_timeDbl);
  void setFootSize(double in_toe, double in_heel, double in_outer, double in_inner);
  void setOffset(double in_dx, double in_dy);  // left foot base
  inline void setStepping(bool in_flag=true) { m_stepping = in_flag; }
  inline bool isStepping() { return m_stepping; }
  inline void setEndFlag(bool in_flag) { m_endIsStop = in_flag; }
  inline bool endIsStop() { return m_endIsStop; }
  inline void setMargin(double in_margin) { m_margin = in_margin; }

  void initParameters(creek::Vector3 in_zmp);

  creek::FootType getFootType(const creek::Vector3 &in_zmp, double in_margin=0);  // zmp (on world)
  creek::FootType getSwingFootType(const creek::Position &rfootRef, const creek::Position &lfootRef);

  void get(creek::StepData &out_step);
  void addStop(bool moveToMidCom=false);
  void addStep(const creek::Position &in_swgRef, creek::FootType in_swgType, int type=0);


  void addStepPlane(const creek::Position &in_swgRef, creek::FootType in_swgType, bool dynamic=true);
  void addStepStairs(const creek::Position &in_swgRef, creek::FootType in_swgType);

  void addComSequence(const creek::Vector3 &in_cpRef, double in_time, creek::FootType in_supType);
  void addStepSequence(const creek::Position &in_swgRef, const creek::Vector3 &in_cpRef, creek::FootType in_swgType);

  inline StepSequence& step() { return m_step; }


  inline void test() {
    //std::cout << m_robot->rfoot()->name() << " : " << m_robot->rfoot()->p().format(creek::IOvec()) << " [m]" << std::endl;
    //std::cout << m_robot->lfoot()->name() << " : " << m_robot->lfoot()->p().format(creek::IOvec()) << " [m]" << std::endl;
  }


private:
  bool needLiftUpFoot(const creek::Position &in_swgRef, const creek::Position &in_swgCur);
  void calcCapturePointParameter(double in_time, double in_ground, double in_com);
  double calcCapturePointParameter(double in_time, creek::FootType supType, double in_com);  // return ground height
  double calcCapturePointParameter(double in_time, creek::FootType supType);  // return ground height


  RobotPtr m_robot;
  StepSequence m_step;

  double m_dt, m_timeSwg, m_timeDbl;
  creek::Vector4 m_footSize; // toe, heel, outer, inner
  creek::Vector3 m_offset;
  bool m_stepping, m_endIsStop;
  double m_margin;  // default margin for getFootType

  double m_w, m_b;
  boost::mutex mtx;
};

#endif
