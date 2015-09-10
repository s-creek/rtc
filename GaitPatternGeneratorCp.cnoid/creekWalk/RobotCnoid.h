// -*- mode: c++; -*-

#ifndef ROBOT_CNOID_H
#define ROBOT_CNOID_H

#include <creekTypesEigen.h>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/Link>

class Robot : public cnoid::Body
{
public:
  Robot();

  inline bool isInit() { return m_isInit; };

  inline void setSoleName(const std::string &in_rsoleName, const std::string &in_lsoleName) {
    m_rsole->setName(in_rsoleName);
    m_lsole->setName(in_lsoleName);
  }
  inline void setSoleOffset(const cnoid::Vector3 &in_roffset, const cnoid::Vector3 &in_loffset) {
    m_rsole->setOffsetTranslation(in_roffset);
    m_lsole->setOffsetTranslation(in_loffset);
    calcForwardKinematics();
  }
  bool setJointPath(const std::string &in_rfootName, const std::string &in_rfootBase, 
		    const std::string &in_lfootName, const std::string &in_lfootBase);

  void updateWaistBase(cnoid::Vector3 &in_waistPos, cnoid::Matrix3 &in_waistRot);
  void updateFootBase(cnoid::Vector3 &in_footPos, cnoid::Matrix3 &in_footRot, creek::FootType in_supFoot);

  // only foot
  bool calc(creek::FootType in_supportFoot, const cnoid::Vector3 &in_comPosRef, const cnoid::Matrix3 &in_waistRotRef,
	    const cnoid::Vector3 &in_rfootPosRef, const cnoid::Matrix3 &in_rfootRotRef,
	    const cnoid::Vector3 &in_lfootPosRef, const cnoid::Matrix3 &in_lfootRotRef);
  bool calcComInverseKinematics(creek::FootType in_supportFoot, const cnoid::Vector3 &in_comPosRef, const cnoid::Matrix3 &in_waistRotRef,
				const cnoid::Vector3 &in_swingPosRef, const cnoid::Matrix3 &in_swingRotRef);
  
  inline cnoid::JointPathPtr rleg() const { return m_wl2rfPath; }
  inline cnoid::JointPathPtr lleg() const { return m_wl2lfPath; }
  inline cnoid::Link* rfoot() const { return m_wl2rfPath->endLink(); }
  inline cnoid::Link* lfoot() const { return m_wl2lfPath->endLink(); }


private:
  bool m_isInit;
  cnoid::Link *m_rsole, *m_lsole;

  // wl : wasit link
  // rf : right foot,  lf : left foot
  cnoid::JointPathPtr m_rf2wlPath, m_lf2wlPath;
  cnoid::JointPathPtr m_wl2rfPath, m_wl2lfPath;
  cnoid::JointPathPtr m_rf2lfPath, m_lf2rfPath;
};

//typedef boost::intrusive_ptr<Robot> RobotPtr;
typedef cnoid::ref_ptr<Robot> RobotPtr;
typedef cnoid::Link Link;

#endif
