#include "WalkPlanner.h"

#include <creekGeometry.h>

using namespace creek;
enum LegType {RLEG=0, LLEG};

WalkPlanner::WalkPlanner()
  : m_stepping(false),
    m_endIsStop(true),
    m_margin(0.0)
{
  // set default parameter
  setTime(0.005, 0.7, 0.1);
  setFootSize(0.1, 0.1, 0.05, 0.05);
  setOffset(0, 0);
}


void WalkPlanner::setTime(double in_dt, double in_timeSwg, double in_timeDbl)
{
  m_dt = in_dt;
  m_timeSwg = in_timeSwg;
  m_timeDbl = in_timeDbl;

  m_step.setTime(in_dt, in_timeSwg, in_timeDbl);
}


void WalkPlanner::setFootSize(double in_toe, double in_heel, double in_outer, double in_inner)
{
  m_footSize << in_toe, in_heel, in_outer, in_inner;
}


void WalkPlanner::setOffset(double in_dx, double in_dy)
{
  m_offset << in_dx, in_dy, 0.0;
}


void WalkPlanner::initParameters(creek::Vector3 in_zmp)
{
  StepData initData;
  initData.rfoot = m_robot->rfoot()->position();
  initData.lfoot = m_robot->lfoot()->position();
  initData.com = m_robot->calcCenterOfMass();
  initData.zmp = in_zmp;
  initData.cp  << initData.com[0], initData.com[1], 0;
  initData.sup = DFOOT;
  
  m_step.init(initData); 


  // set default com height
  double ground = m_step.getGroundHeight(creek::DFOOT);
  m_step.comHeight() = initData.com[2] - ground;
}


FootType WalkPlanner::getFootType(const Vector3 &in_zmp, double in_margin)
{
  FootType ret(AIR);

  if( m_step.empty() ) {
    initParameters(in_zmp);
  }
  Vector3  com   = m_step.back().com;
  Position rfoot = m_step.back().rfoot;
  Position lfoot = m_step.back().lfoot;

  Vector3 relate(in_zmp-com);
  double  dz = relate[2];  // for DOUBLE
  if( fabs(dz) < 1.0e-12 )
    return ret;


  // RIGHT
  if( ret == AIR ) {
    Vector3 n(rfoot.linear()*Vector3(0,0,1));
    Vector4 plane;
    getPlaneEquation(rfoot.translation(), n, plane);
    
    Vector3 rzmp;
    if( getIntersectPlaneAndLine(plane, com, in_zmp, rzmp) ) {
      
      Vector3 distR;
      distR = rfoot.linear().transpose() * (rzmp-rfoot.translation());
      if( distR(0) < (m_footSize(0)-in_margin)      // toe
	  && distR(0) > -(m_footSize(1)-in_margin)  // heel
	  && distR(1) > -(m_footSize(2)-in_margin)  // outer
	  && distR(1) < m_footSize(3))              // inner
	ret = RFOOT;
    }
  }

  // LEFT
  if( ret == AIR ) {
    Vector3 n(lfoot.linear()*Vector3(0,0,1));
    Vector4 plane;
    getPlaneEquation(lfoot.translation(), n, plane);
    
    Vector3 lzmp;
    if( getIntersectPlaneAndLine(plane, com, in_zmp, lzmp) ) {

      Vector3 distL;
      distL = lfoot.linear().transpose() * (lzmp-lfoot.translation());
      if( distL(0) < (m_footSize(0)-in_margin)      // toe
	  && distL(0) > -(m_footSize(1)-in_margin)  // heel
	  && distL(1) < (m_footSize(2)-in_margin)   // outer
	  && distL(1) > -m_footSize(3))             // inner
	ret = LFOOT;
    }
  }

  // DOUBLE
  if( ret == AIR ) {
    Vector3 rt, rh, lt, lh;  // RorL, TOEorHEEL
    rt = rfoot.translation() + rfoot.linear() * Vector3(m_footSize(0)-in_margin, m_footSize(3), 0.0) - com;
    rh = rfoot.translation() + rfoot.linear() * Vector3(-m_footSize(1)+in_margin, m_footSize(3), 0.0) - com;
    lt = lfoot.translation() + lfoot.linear() * Vector3(m_footSize(0)-in_margin, -m_footSize(3), 0.0) - com;
    lh = lfoot.translation() + lfoot.linear() * Vector3(-m_footSize(1)+in_margin, -m_footSize(3), 0.0) - com;

    // projection
    rt = rt / rt[2] * dz + com;
    rh = rh / rh[2] * dz + com;
    lt = lt / lt[2] * dz + com;
    lh = lh / lh[2] * dz + com;

    Vector3 rt2lt(lt-rt), rt2rh(rh-rt), lh2lt(lt-lh), lh2rh(rh-lh);
    Vector3 rt2zmp(in_zmp-rt), lh2zmp(in_zmp-lh);
    if( rt2lt.cross(rt2zmp)[2] >= 0.0 
	&& rt2rh.cross(rt2zmp)[2] <= 0.0 
	&& lh2rh.cross(lh2zmp)[2] >= 0.0
	&& lh2lt.cross(lh2zmp)[2] <= 0.0)
      ret = DFOOT;
  }

  return ret;
}


creek::FootType WalkPlanner::getSwingFootType(const creek::Position &rfootRef, const creek::Position &lfootRef)
{
  creek::FootType ret(creek::DFOOT);

  if( m_step.empty() )
    return ret;

  bool mr = needLiftUpFoot(rfootRef, m_step.back().rfoot);
  bool ml = needLiftUpFoot(lfootRef, m_step.back().lfoot);
  if( mr && ml ) {
    ret = creek::DFOOT;
  }
  else if( mr ) {
    ret = creek::RFOOT;
  }
  else if( ml ) {
    ret = creek::LFOOT;
  }
  else {
    ret = creek::AIR;
  }
  return ret;
}


void WalkPlanner::addStop(bool moveToMidCom)
{
  if( m_endIsStop || m_step.empty() )
    return;

  std::cout << "WalkPlanner : addStop" << std::endl;
  m_endIsStop = true;
  double time = m_step.stepTime();


  // set current and reference data
  creek::Vector3 comRef(m_step.back().com), comCur(m_step.back().com);
  if( moveToMidCom ) {
    comRef = 0.5 * ( m_step.back().rfoot.translation() + m_step.back().lfoot.translation() );
    comRef[2] = comCur[2];
  }

  creek::Vector3 zmpRef(comRef);
  zmpRef[2] = m_step.back().zmp[2];

  creek::Vector3 cpRef(comRef), cpCur(m_step.back().cp);
  cpRef[2] = m_step.back().zmp[2];;


  // check com velocity
  creek::Vector3 comVelCur, comVelRef(0,0,0);
  comVelCur = m_w * (cpCur - comCur);
  comVelCur[2] = 0.0;
  if( comVelCur.norm() < 0.05 )
    return;


  //
  // first half deceleration
  //
  addComSequence(cpRef, time, creek::DFOOT);
  

  // these value changed in first deceleration
  comCur = m_step.back().com;  
  cpCur  = m_step.back().cp;
  comVelCur = m_w * (cpCur - comCur);
  comVelCur[2] = 0.0;
  if( comVelCur.norm() < 0.05 )
    return;


  //
  // second half deceleration
  //
  if(false) {
    // capture point ver
    addComSequence(cpRef, time, creek::DFOOT);
  }
  else {
    // cubic interpolation ver
    std::deque<creek::Vector4> cubicParam(2);
    for(int i=0; i<2; i++) {
      cubicParam.at(i)[0] = comCur[i];
      cubicParam.at(i)[1] = comVelCur[i];
      cubicParam.at(i)[2] = ( 3*(comRef[i]-comCur[i])/time - 2*comVelCur[i] - comVelRef[i] )/time;
      cubicParam.at(i)[3] = ( 2*(comCur[i]-comRef[i])/time + comVelCur[i] + comVelRef[i] )/time/time;
    }


    // add fott
    int start = m_step.size();
    m_step.wait(time, creek::DFOOT);


    // fix zmp, cp
    //     com (cubic interpolation)
    int count=1;
    for(int i=start; i<m_step.size(); i++) {
      time = m_dt * count;
      count++;
      try {
	m_step.at(i).com[0] = cubicParam.at(0)[0] + ( cubicParam.at(0)[1] + ( cubicParam.at(0)[2] + cubicParam.at(0)[3]*time )*time )*time;
	m_step.at(i).com[1] = cubicParam.at(1)[0] + ( cubicParam.at(1)[1] + ( cubicParam.at(1)[2] + cubicParam.at(1)[3]*time )*time )*time;
	
	m_step.at(i).zmp = zmpRef;
	m_step.at(i).cp  = cpRef;
      }
      catch(std::out_of_range e) {
	std::cerr << "WalkPlanner : addStop" << std::endl << "  " << e.what() << std::endl;
	std::cerr << "  size = " << m_step.size() << ",  i = " << i << std::endl;
      }
    }
    m_step.back().com = comRef;
  }
}


void WalkPlanner::get(creek::StepData &out_step)
{
  boost::mutex::scoped_lock lock(mtx);
  m_step.get(out_step);
}


void WalkPlanner::addStep(const creek::Position &in_swgRef, creek::FootType in_swgType, int type)
{
  boost::mutex::scoped_lock lock(mtx);
  switch( type )
    {
    case 1:
      addStepPlane(in_swgRef, in_swgType, false);
      break;
    case 2:
      addStepStairs(in_swgRef, in_swgType);
    case 0:
    default:
      addStepPlane(in_swgRef, in_swgType, true);
    }
}


void WalkPlanner::addStepPlane(const creek::Position &in_swgRef, creek::FootType in_swgType, bool dynamic)
{
  if( m_step.empty() || in_swgType == AIR )
    return;


  if( in_swgType == DFOOT ) {
    // todo : anything
    return;
  }


  //
  // set parameters
  //
  FootType supType(in_swgType);
  Position swgPos, supPos;  // end sequence data
  Vector3  swgOffset(m_offset), supOffset(m_offset);

  if( in_swgType == RFOOT ) {
    supType = LFOOT;
    swgPos = m_step.back().rfoot;
    supPos = m_step.back().lfoot;
    swgOffset[1] = -swgOffset[1];  // rfoot
  }
  else if( in_swgType == LFOOT ) {
    supType = RFOOT;
    swgPos = m_step.back().lfoot;
    supPos = m_step.back().rfoot;
    supOffset[1] = -supOffset[1];  // rfoot
  }


  //
  // check status
  //
  bool liftup = needLiftUpFoot(in_swgRef, swgPos);
  if( liftup ) {
    m_endIsStop = false;
  }
  else if( !liftup && m_endIsStop ) {
    return;
  }


  //
  //  ref zmp in support foot ?
  //
  Vector3 zmpRef, cpRef, cp, com;
  cp  = m_step.back().cp;
  com = m_step.back().com;
  if( dynamic ) {
    cpRef = in_swgRef.translation() + in_swgRef.linear() * swgOffset;
  }
  else {
    cpRef = supPos.translation() + supPos.linear() * supOffset;
  }
  double ground = calcCapturePointParameter( m_step.stepTime(), supType );
  zmpRef = 1/(1-m_b)*cpRef - m_b/(1-m_b)*cp;
  zmpRef[2] = ground;
  FootType supTypeExpected = getFootType(zmpRef, m_margin);

  bool moveCom(false);  // move com before step
  if( liftup && supTypeExpected != supType ) {
    std::cout << "WalkPlanner : addStep" << std::endl;
    std::cout << "  expected sup type " << supTypeExpected << ",  input sup type = " << supType << std::endl;
    std::cout << "  zmp ref = " << zmpRef.format(IOvec()) << std::endl;
    std::cout << "  sup pos = " << supPos.translation().format(IOvec()) << std::endl;
    std::cout << "  com pos = " << com.format(IOvec()) << std::endl;
    std::cout << "  ground  = " << ground << std::endl;
    moveCom = true;
  }

  
  //
  // add com sequence
  //
  if( moveCom ) {
    Vector3 cpRefCom(cpRef);
    if( dynamic )
      cpRefCom = supPos.translation() + supPos.linear() * supOffset;
    
    double a(0.02);  // margin : 0.02 [m]
    double bx, by;
    bx = 1 + fabs( (cpRefCom[0]-cp[0])/a );
    by = 1 + fabs( (cpRefCom[1]-cp[1])/a );
    
    double b = std::max(bx, by);
    double timeMoveCom = std::log(b) / m_w;
    int n = timeMoveCom / m_dt;
    timeMoveCom = n*m_dt + 0.15;  // margin : 0.15 [s]
   
    addComSequence(cpRefCom, timeMoveCom, supType);
  }


  //
  // add step sequence
  //
  if( liftup ) {
    addStepSequence(in_swgRef, cpRef, in_swgType);
  }
  else {
    addComSequence(cpRef, m_step.stepTime(), supType);
  }


  return;
}


void WalkPlanner::addStepStairs(const creek::Position &in_swgRef, creek::FootType in_swgType)
{
  // todo
}


void WalkPlanner::addComSequence(const creek::Vector3 &in_cpRef, double in_time, creek::FootType in_supType)
{
  //std::cout << "addComSequence" << std::endl;
  if( m_step.empty() )
    return;


  Vector3 com = m_step.back().com;
  Vector3 cp  = m_step.back().cp;
  cp[2] = 0.0;

  double ground = calcCapturePointParameter(in_time, in_supType, com[2]);
  Vector3 zmp = 1/(1-m_b)*in_cpRef - m_b/(1-m_b)*cp;
  zmp[2] = ground;


  // add foot
  int start = m_step.size();
  m_step.wait(in_time);


  // fix com, zmp, cp (capture point)
  double wdt(m_w*m_dt);
  for(int i=start; i<m_step.size(); i++) {
    cp  = std::exp(wdt)*cp + (1-std::exp(wdt))*zmp;
    cp[2] = 0.0;
    com = std::exp(-wdt)*com + (1-std::exp(-wdt))*cp;
  
    try {
      m_step.at(i).zmp = zmp;
      m_step.at(i).com[0] = com[0];
      m_step.at(i).com[1] = com[1];
      m_step.at(i).cp  = cp;
    }
    catch(std::out_of_range e) {
      std::cerr << "WalkPlanner : addComSequence" << std::endl << "  " << e.what() << std::endl;
      std::cerr << "  size = " << m_step.size() << ",  i = " << i << std::endl;
    }
  }
}


void WalkPlanner::addStepSequence(const creek::Position &in_swgRef, const creek::Vector3 &in_cpRef, creek::FootType in_swgType)
{
  //std::cout << "addStepSequence" << std::endl;
  if( m_step.empty() )
    return;


  // check swing foot type
  creek::FootType supType;
  if( in_swgType == creek::RFOOT ) {
    supType = creek::LFOOT;
  }
  else if( in_swgType == creek::LFOOT ) {
    supType = creek::RFOOT;
  }
  else {
    return;
  }


  Vector3 com = m_step.back().com;
  Vector3 cp  = m_step.back().cp;
  cp[2] = 0.0;

  double ground = calcCapturePointParameter(m_step.stepTime(), supType);
  Vector3 zmp = 1/(1-m_b)*in_cpRef - m_b/(1-m_b)*cp;
  zmp[2] = ground;


  // add foot
  int start = m_step.size();
  m_step.add(in_swgRef, in_swgType);
  

  // fix com, zmp, cp (capture point)
  double wdt(m_w*m_dt);
  for(int i=start; i<m_step.size(); i++) {
    cp  = std::exp(wdt)*cp + (1-std::exp(wdt))*zmp;
    cp[2] = 0.0;
    com = std::exp(-wdt)*com + (1-std::exp(-wdt))*cp;
  
    try {
      m_step.at(i).zmp = zmp;
      m_step.at(i).com[0] = com[0];
      m_step.at(i).com[1] = com[1];
      m_step.at(i).cp  = cp;
    }
    catch(std::out_of_range e) {
      std::cerr << "WalkPlanner : addStepSequence" << std::endl << "  " << e.what() << std::endl;
      std::cerr << "  size = " << m_step.size() << ",  i = " << i << std::endl;
    }
  }
}


bool WalkPlanner::needLiftUpFoot(const creek::Position &in_swgRef, const creek::Position &in_swgCur)
{
  if( m_stepping )
    return true;


  double dpmin(1.0e-6), dRmin(1.0e-6);
  bool need(false);
 
  Vector3 dp = in_swgRef.translation() - in_swgCur.translation();
  Matrix3 dR = in_swgRef.linear().transpose() * in_swgCur.linear();
  Eigen::AngleAxisd omega(dR);
  if( dp.norm() > dpmin || fabs(omega.angle()) > dRmin )
    need = true; 

  return need;
}


void WalkPlanner::calcCapturePointParameter(double in_time, double in_ground, double in_com)
{
  static double g(9.80665);
  double z = in_com - in_ground;

  m_w = std::sqrt(g/z);
  m_b = std::exp(m_w*in_time);
}


double WalkPlanner::calcCapturePointParameter(double in_time, creek::FootType supType, double in_com)
{
  double ground = m_step.getGroundHeight(supType);
  calcCapturePointParameter(in_time, ground, in_com);
  return ground;
}


double WalkPlanner::calcCapturePointParameter(double in_time, creek::FootType supType)
{
  // calc capture point parameter
  static double g(9.80665);
  double z = m_step.comHeight();

  m_w = std::sqrt(g/z);
  m_b = std::exp(m_w*in_time);


  // calc ground height
  double ground = m_step.getGroundHeight(supType);
  return ground;
}
