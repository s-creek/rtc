// -*- c++ -*-

#ifndef STEP_SEQUENCE_H
#define STEP_SEQUENCE_H

#include <creekTypesEigen.h>
#include <deque>

class StepSequence
{
public:
  StepSequence();
  ~StepSequence();

  void setTime(double in_dt, double in_timeSwg, double in_timeDbl);
  inline double stepTime() { return (m_timeSwg+m_timeDbl); }
  inline void setZup(double in_zup) { m_zup=in_zup; }
  inline double comHeight() const { return m_comHeight; }
  inline double& comHeight() { return m_comHeight; }

  void init(creek::StepData in_step);
  void get(creek::StepData &out_step);

  void wait(double in_time);
  void wait(double in_time, creek::FootType in_supType);
  void add(const creek::Position &in_swgRef, creek::FootType in_swgType, int mode=0);  // mode : 0=plane, 1=stairs

  void plane(const creek::Position &in_swgRef, creek::FootType in_swgType);
  void stairs(const creek::Position &in_swgRef, creek::FootType in_swgType);

  double getGroundHeight(creek::FootType in_supType);
  
  inline std::deque<creek::StepData>& sequence() { return m_seqStep; }
  inline bool empty() { return m_seqStep.empty(); }
  inline void clear() { m_seqStep.clear(); }
  inline std::deque<creek::StepData>::size_type size() { return m_seqStep.size(); }
  inline creek::StepData& at(int i) { return m_seqStep.at(i); }
  inline creek::StepData& back() { return m_seqStep.back(); }
  inline creek::StepData& front() { return m_seqStep.front(); }


private:
  void moveRfoot(const creek::Position &in_swgRef, double comRef);
  void moveLfoot(const creek::Position &in_swgRef, double comRef);
  void calcRotationInterpolatorParameter(const creek::Matrix3 &refR, const creek::Matrix3 &curR, 
					 Eigen::AngleAxisd &angleAxisA, Eigen::AngleAxisd &angleAxisB);

  double m_dt, m_timeSwg, m_timeDbl;
  double m_zup;
  double m_comHeight;

  std::deque<creek::StepData> m_seqStep;
};

#endif
