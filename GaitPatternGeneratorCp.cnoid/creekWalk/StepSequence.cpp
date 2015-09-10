#include "StepSequence.h"

StepSequence::StepSequence()
  : m_zup(0.05)
{
  setTime(0.005, 0.65, 0.10);
}


StepSequence::~StepSequence()
{
}


void StepSequence::setTime(double in_dt, double in_timeSwg, double in_timeDbl)
{
  m_dt = in_dt;
  m_timeSwg = in_timeSwg;
  m_timeDbl = in_timeDbl;
}


void StepSequence::init(creek::StepData in_step)
{
  m_seqStep.clear();
  m_seqStep.push_back(in_step);
}


void StepSequence::get(creek::StepData &out_step)
{
  if( empty() )
    return;
  
  out_step = m_seqStep.front();
  if( !m_seqStep.empty() )
    m_seqStep.pop_front();
}


void StepSequence::wait(double in_time)
{
  if( empty() )
    return;

  creek::FootType supType = m_seqStep.back().sup;
  wait(in_time, supType);
}


void StepSequence::wait(double in_time, creek::FootType in_supType)
{
  if( empty() )
    return;
  
  creek::StepData data = m_seqStep.back();
  data.sup = in_supType;

  int n = (in_time/m_dt+1.0e-6);
  for(int i=0; i<n; i++) {
    m_seqStep.push_back(data);
  }
}


void StepSequence::add(const creek::Position &in_swgRef, creek::FootType in_swgType, int mode)
{
  if( empty() )
    return;

  if( mode == 0 ) {
    plane(in_swgRef, in_swgType);
  }
  else {
    stairs(in_swgRef, in_swgType);
  }
}


void StepSequence::plane(const creek::Position &in_swgRef, creek::FootType in_swgType)
{
  if( empty() )
    return;

  creek::FootType supType(creek::DFOOT);
  double ground = getGroundHeight(creek::DFOOT);
  if( in_swgType == creek::RFOOT ) {
    supType = creek::LFOOT;
    ground = std::min( in_swgRef.translation()[2], m_seqStep.back().lfoot.translation()[2] );
  }
  else if( in_swgType == creek::LFOOT ){
    supType = creek::RFOOT;
    ground = std::min( in_swgRef.translation()[2], m_seqStep.back().rfoot.translation()[2] );
  }
  double comRef = ground + m_comHeight;
  //std::cout << "StepSequence : com ref height = " << comRef << std::endl;


  // first half double support phase
  wait(m_timeDbl/2.0, creek::DFOOT);


  // single support phase
  if( in_swgType == creek::RFOOT ) {
    moveRfoot(in_swgRef, comRef);
  }
  else if( in_swgType == creek::LFOOT ){
    moveLfoot(in_swgRef, comRef);
  }


  // second half double support phase
  wait(m_timeDbl/2.0, creek::DFOOT);
}


void StepSequence::stairs(const creek::Position &in_swgRef, creek::FootType in_swgType)
{
  if( empty() )
    return;

  // todo
}


void StepSequence::moveRfoot(const creek::Position &in_swgRef, double comRef)
{
  int n = (m_timeSwg/m_dt + 1.0e-6);
  creek::StepData data = m_seqStep.back();  // start data


  // for position
  double dtheta = 2*M_PI/n;
  creek::Vector3 sp(data.rfoot.translation());
  creek::Vector3 gp(in_swgRef.translation());


  // for orientation
  creek::Matrix3 sR(data.rfoot.linear());
  creek::Matrix3 gR(in_swgRef.linear());
  Eigen::AngleAxisd aaa, aab;
  calcRotationInterpolatorParameter(gR, sR, aaa, aab);


  // for com pos z
  double comCur = m_seqStep.back().com[2];


  // single support phase
  data.sup = creek::LFOOT;
  for(int i=1; i<n; i++) {
    double theta = i*dtheta;
    double rate = (theta - sin(theta)) / (2*M_PI);

    // position
    {
      // cycloid
      creek::Vector3 dz( 0, 0, m_zup*0.5*(1-cos(theta)) );    
      data.rfoot.translation() = sp + rate*(gp - sp) + dz;
    }

    // orientation
    {
      double angleA = rate * aaa.angle();
      double angleB = rate * aab.angle();
      data.rfoot.linear() = sR * Eigen::AngleAxisd(angleA, aaa.axis() ) * Eigen::AngleAxisd(angleB, aab.axis());
    }

    // com pos z
    {
      data.com[2] = comCur + rate * (comRef - comCur);
    }

    m_seqStep.push_back(data);
  }
  data.rfoot  = in_swgRef;
  data.com[2] = comRef;
  m_seqStep.push_back(data);
}


void StepSequence::moveLfoot(const creek::Position &in_swgRef, double comRef)
{
  int n = (m_timeSwg/m_dt + 1.0e-6);
  creek::StepData data = m_seqStep.back();  // start data


  // for position
  double dtheta = 2*M_PI/n;
  creek::Vector3 sp(data.lfoot.translation());
  creek::Vector3 gp(in_swgRef.translation());


  // for orientation
  creek::Matrix3 sR(data.lfoot.linear());
  creek::Matrix3 gR(in_swgRef.linear());
  Eigen::AngleAxisd aaa, aab;
  calcRotationInterpolatorParameter(gR, sR, aaa, aab);


  // for com pos z
  double comCur = m_seqStep.back().com[2];


  // single support phase
  data.sup = creek::RFOOT;
  for(int i=1; i<n; i++) {
    double theta = i*dtheta;
    double rate = (theta - sin(theta)) / (2*M_PI);

    // position
    {
      // cycloid
      creek::Vector3 dz( 0, 0, m_zup*0.5*(1-cos(theta)) );
      data.lfoot.translation() = sp + rate*(gp - sp) + dz;
    }

    // orientation
    {
      double angleA = rate * aaa.angle();
      double angleB = rate * aab.angle();
      data.lfoot.linear() = sR * Eigen::AngleAxisd(angleA, aaa.axis() ) * Eigen::AngleAxisd(angleB, aab.axis());
    }

    // com pos z
    {
      data.com[2] = comCur + rate * (comRef - comCur);
    }

    m_seqStep.push_back(data);
  }
  data.lfoot  = in_swgRef;
  data.com[2] = comRef;
  m_seqStep.push_back(data);
}


void StepSequence::calcRotationInterpolatorParameter(const creek::Matrix3 &refR, const creek::Matrix3 &curR,
						     Eigen::AngleAxisd &angleAxisA, Eigen::AngleAxisd &angleAxisB)
{
  creek::Vector3 ez(0,0,1);


  // 誤差回転行列Rの導出
  creek::Matrix3 dR( curR.transpose() * refR );


  // target方向への回転の導出
  double cos_angle_a = ez.dot( dR * ez );
  double angle_a;
  if( cos_angle_a > (1.0 - 1.0e-6) )
    angle_a = 0;
  else if(cos_angle_a < -(1.0 - 1.0e-6) )
    angle_a = M_PI;
  else
    angle_a = acos(cos_angle_a);


  creek::Vector3 axis_a;
  if(angle_a > 1e-3){
    axis_a = ez.cross( dR * ez );
    axis_a.normalize();
  }
  else{
    axis_a = creek::Vector3(1,0,0);
  }
  axis_a.normalize();

  angleAxisA.angle() = angle_a;
  angleAxisA.axis()  = axis_a;


  // target方向以外への回転の導出
  creek::Matrix3 R_a( Eigen::AngleAxisd(angle_a, axis_a) );
  creek::Matrix3 R_b( R_a.transpose() * dR );
  angleAxisB.fromRotationMatrix(R_b);
}


double StepSequence::getGroundHeight(creek::FootType in_supType)
{
  if( empty() )
    return 0;


  if( in_supType == creek::RFOOT ) {
    return m_seqStep.back().rfoot.translation()[2];
  }
  else if( in_supType == creek::LFOOT ) {
    return m_seqStep.back().lfoot.translation()[2];
  }
  else {
    if( m_seqStep.back().rfoot.translation()[2] < m_seqStep.back().lfoot.translation()[2] ) {
      return getGroundHeight(creek::RFOOT);
    }
    else {
      return getGroundHeight(creek::LFOOT);
    }
  }
}
