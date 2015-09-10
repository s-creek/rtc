// -*- mode: c++; -*-

#include "RobotCnoid.h"
#include <iostream>
#include <cnoid/EigenUtil>
#include <cnoid/Jacobian>

using namespace cnoid;
typedef Eigen::MatrixXd dmatrix;
typedef Eigen::VectorXd dvector;

Robot::Robot()
  : m_isInit(false)
{
  m_rsole = new Link();
  m_lsole = new Link();

  setSoleName("CREEK_RSOLE", "CREEK_LSOLE");
}


bool Robot::setJointPath(const std::string &in_rfootName, const std::string &in_rfootBase, const std::string &in_lfootName, const std::string &in_lfootBase)
{
  m_isInit = true;

  // add sole
  Body::link(in_rfootName)->appendChild(m_rsole);
  Body::link(in_lfootName)->appendChild(m_lsole);
  Body::updateLinkTree();

  // get joint path
  m_rf2wlPath = getCustomJointPath(this, m_rsole, Body::link(in_rfootBase));
  if( m_rf2wlPath->empty() ) {
    std::cerr << "Robot : get jointpath (RFOOT->ROOT)" << std::endl;
    m_isInit = false;
  }

  m_lf2wlPath = getCustomJointPath(this, m_lsole, Body::link(in_lfootBase));
  if( m_lf2wlPath->empty() ) {
    std::cerr << "Robot : get jointpath (LFOOT->ROOT)" << std::endl;
    m_isInit = false;
  }

  m_wl2rfPath = getCustomJointPath(this, Body::link(in_rfootBase), m_rsole);
  if( m_wl2rfPath->empty() ) {
    std::cerr << "Robot : get jointpath (ROOT->RFOOT)" << std::endl;
    m_isInit = false;
  }

  m_wl2lfPath = getCustomJointPath(this, Body::link(in_lfootBase), m_lsole);
  if( m_wl2lfPath->empty() ) {
    std::cerr << "Robot : get jointpath (ROOT->LFOOT)" << std::endl;
    m_isInit = false;
  }

  m_rf2lfPath = getCustomJointPath(this, m_rsole, m_lsole);
  if( m_rf2lfPath->empty() )  {
    std::cerr << "Robot : get jointpath (RFOOT->LFOOT)" << std::endl;
    m_isInit = false;
  }

  m_lf2rfPath = getCustomJointPath(this, m_lsole, m_rsole);
  if( m_lf2rfPath->empty() )  {
    std::cerr << "Robot : get jointpath (LFOOT->RFOOT)" << std::endl;
    m_isInit = false;
  }

  return m_isInit;
}


void Robot::updateWaistBase(Vector3 &in_waistPos, Matrix3 &in_waistRot)
{
  Body::rootLink()->p() = in_waistPos;
  Body::rootLink()->R() = in_waistRot;
  Body::calcForwardKinematics();
}


void Robot::updateFootBase(Vector3 &in_footPos, Matrix3 &in_footRot, creek::FootType in_supFoot)
{
  if( in_supFoot < 0 ) {
    Robot::rfoot()->p() = in_footPos;
    Robot::rfoot()->R() = in_footRot;
    m_rf2wlPath->calcForwardKinematics();
  }
  else {
    Robot::lfoot()->p() = in_footPos;
    Robot::lfoot()->R() = in_footRot;
    m_lf2wlPath->calcForwardKinematics();
  }
  Body::calcForwardKinematics();
}


bool Robot::calc(creek::FootType in_supportFoot, const Vector3 &in_comPosRef, const Matrix3 &in_waistRotRef,
		 const Vector3 &in_rfootPosRef, const Matrix3 &in_rfootRotRef,
		 const Vector3 &in_lfootPosRef, const Matrix3 &in_lfootRotRef)
{
  bool converged = false;


  // for error
  int n = Body::numJoints();
  std::vector<double> qorg(n);
  for(int i=0; i < n; ++i) {
    qorg[i] = Body::joint(i)->q();
  }
  Vector3 porg(Body::rootLink()->p());
  Matrix3 Rorg(Body::rootLink()->R());


  // calc inverse
  Body::calcForwardKinematics();
  if( in_supportFoot == creek::RFOOT ) {
    if( !m_wl2rfPath->calcInverseKinematics(in_rfootPosRef, in_rfootRotRef) )
      return false;

    converged = calcComInverseKinematics(in_supportFoot, in_comPosRef, in_waistRotRef, in_lfootPosRef, in_lfootRotRef);
  }
  // とりあえず両脚支持の時も左足を支持脚に
  else {
    if( !m_wl2lfPath->calcInverseKinematics(in_lfootPosRef, in_lfootRotRef) )
      return false;

    in_supportFoot = creek::LFOOT;
    converged = calcComInverseKinematics(in_supportFoot, in_comPosRef, in_waistRotRef, in_rfootPosRef, in_rfootRotRef);
  }


  if( !converged ) {
    Body::rootLink()->p() = porg;
    Body::rootLink()->R() = Rorg;
    for(int i=0; i < n; ++i) {
      Body::joint(i)->q() = qorg[i];
    }
    Body::calcForwardKinematics();
  }


  return converged;
}


bool Robot::calcComInverseKinematics(creek::FootType in_supportFoot, const Vector3 &in_comPosRef, const Matrix3 &in_waistRotRef,
				     const Vector3 &in_swingPosRef, const Matrix3 &in_swingRotRef)
{
  if( !m_isInit ) {
    std::cerr << "Robot : please setJointPath()" << std::endl;
    return false;
  }
  

  //
  // set variables
  //
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;


  //
  // set joint path
  //
  // sp : support,  sw : swing
  JointPathPtr sp2wlPath, sp2swPath;
  if( in_supportFoot == creek::RFOOT ) {
    sp2wlPath = m_rf2wlPath;
    sp2swPath = m_rf2lfPath;
  }
  else if( in_supportFoot == creek::LFOOT ) {
    sp2wlPath = m_lf2wlPath;
    sp2swPath = m_lf2rfPath;
  }
  else {
    return false;
  }
  Position supBase = sp2swPath->baseLink()->position();


  int n = sp2swPath->numJoints();
  std::vector<double> qorg(n);
  for(int i=0; i < n; ++i) {
    qorg[i] = sp2swPath->joint(i)->q();
  }
  

  //
  // set val conf
  //
  dmatrix J_com(3, Body::numJoints());
  dmatrix J_sup(6, sp2wlPath->numJoints());
  dmatrix J_leg(6, n);
  dmatrix J(12, n);
  dmatrix JJ;
  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  double dampingConstantSqr=1e-12;

  dvector v(12);
  dvector dq(n);

  double maxIKErrorSqr = 1.0e-16;
  double errsqr = maxIKErrorSqr * 100.0;
  bool converged = false;


  //
  // main loop
  //
  for(int k=0; k < MAX_IK_ITERATION; k++) {
    //
    // calc velocity
    //
    Vector3 comPosCur(Body::calcCenterOfMass());
    Vector3 dComPos(in_comPosRef-comPosCur);
    Vector3 dWaistOmega(Body::rootLink()->R()*omegaFromRot(Matrix3(Body::rootLink()->R().transpose()*in_waistRotRef)));
    Vector3 dSwingPos(in_swingPosRef-sp2swPath->endLink()->p());
    Vector3 dSwingOmega(sp2swPath->endLink()->R()*omegaFromRot(Matrix3(sp2swPath->endLink()->R().transpose()*in_swingRotRef)));

    v.head<3>()     = dComPos;
    v.segment<3>(3) = dWaistOmega;
    v.segment<3>(6) = dSwingPos;
    v.segment<3>(9) = dSwingOmega;

    errsqr = v.squaredNorm();
    if( errsqr < maxIKErrorSqr ) {
      converged = true;
      break;
    }
    else if( MAX_IK_ITERATION == 1) {
      converged = true;
    }
      


    //
    // calc jacobians
    //
    Link*   base = sp2swPath->baseLink();
    calcCMJacobian(this, base, J_com);  // calc com jacobian
    sp2wlPath->calcJacobian(J_sup);     // calc sup jacobian
    sp2swPath->calcJacobian(J_leg);     // calc legs jacobian
    

    // calc all Jacobian
    J = Eigen::MatrixXd::Zero(12,n);

    // set com jacobian
    for(int i = 0; i < sp2swPath->numJoints(); i++) {
      int id = sp2swPath->joint(i)->jointId();
      for(int j = 0; j < 3; j++) {
	J(j,i) = J_com(j,id);
      }
    }

    // set sup jacobian
    for(int i = 0; i < sp2wlPath->numJoints(); i++) {
      for(int j = 3; j < 6; j++) {
	J(j,i) = J_sup(j,i);
      } 
    }

    // set legs jacobian
    for(int i = 0; i < sp2swPath->numJoints(); i++) {
      for(int j = 0; j < 6; j++) {
	J(j+6,i) = J_leg(j,i);
      } 
    }


    //
    // solve quation
    //
    JJ = J *  J.transpose() + dampingConstantSqr * MatrixXd::Identity(J.rows(), J.rows());
    dq = J.transpose() * QR.compute(JJ).solve(v);
  
    for(int i = 0; i < n; i++) {
      sp2swPath->joint(i)->q() += LAMBDA * dq(i);
    }
    sp2swPath->baseLink()->position() = supBase;
    sp2swPath->calcForwardKinematics();
    Body::calcForwardKinematics();
  }


  if(!converged){
    for(int i=0; i < n; ++i){
      sp2swPath->joint(i)->q() = qorg[i];
    }
    sp2swPath->baseLink()->position() = supBase;
    sp2swPath->calcForwardKinematics();
    Body::calcForwardKinematics();
  }

  return converged;
}
