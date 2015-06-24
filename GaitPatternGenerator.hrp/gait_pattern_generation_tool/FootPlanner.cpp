// -*- c++ -*-

#include "FootPlanner.h"

#include <QueueInterpolator.h>

#define NEAR_ZERO 1.0e-12

using namespace creek;

typedef tvmet::Matrix<double, 3, 3> Matrix33;
typedef tvmet::Vector<double, 3> Vector3;


FootPlanner::FootPlanner(double in_dt)
  : m_dt(in_dt)
{
  m_is_init = false;
  m_end_is_stop = true;
  m_ref_zmp = 0.0;
  m_pre_origin = 0.0;


  m_seq_zmp = new QueueInterpolator(3, in_dt, creek::CUBIC);

  m_seq_rfoot = new QueueInterpolator(3, in_dt, creek::CUBIC);
  m_seq_rfoot_z = new QueueInterpolator(1, in_dt, creek::CUBIC);
  
  m_seq_lfoot = new QueueInterpolator(3, in_dt, creek::CUBIC);
  m_seq_lfoot_z = new QueueInterpolator(1, in_dt, creek::CUBIC);

  m_seq_sup = new QueueInterpolator(1, in_dt, creek::LINEAR);
}


FootPlanner::~FootPlanner()
{
  delete m_seq_zmp;
  delete m_seq_rfoot;
  delete m_seq_rfoot_z;
  delete m_seq_lfoot;
  delete m_seq_lfoot_z;
  delete m_seq_sup;
}


void FootPlanner::setTimeParameter(double in_t_zmp, double in_t_wait, double in_t_land, double in_t_swg)
{
  m_t_zmp  = in_t_zmp;
  m_t_wait = in_t_wait;
  m_t_land = in_t_land;
  m_t_swg  = in_t_swg;
}


void FootPlanner::setFootParameter(double in_to_foot_x, double in_to_foot_y, double in_to_foot_z, double in_to_zmp_x, double in_to_zmp_y)
{
  m_to_rfoot = Vector3(in_to_foot_x, -in_to_foot_y, in_to_foot_z);
  m_to_rzmp  = Vector3(in_to_zmp_x, -in_to_zmp_y, 0);

  m_to_lfoot = Vector3(in_to_foot_x, in_to_foot_y, in_to_foot_z);
  m_to_lzmp  = Vector3(in_to_zmp_x, in_to_zmp_y, 0);
}


void FootPlanner::calcFootParameter(const Vector3 &in_origin, const Vector3 &in_rfoot_p, const Vector3 &in_lfoot_p)
{
  m_to_rfoot = in_rfoot_p - in_origin;
  m_to_lfoot = in_lfoot_p - in_origin;

  m_pre_origin = in_origin;
}


void FootPlanner::init(const Vector3 &in_rfoot_pos, const Matrix33 &in_rfoot_rot, const Vector3 &in_lfoot_pos, const Matrix33 &in_lfoot_rot, const Vector3 &in_zmp, double in_wait_time, FootType in_first_step)
{
  if( m_is_init )  return;
  

  //
  // wait for init pos
  //
  // init right foot
  double rfoot_xyth[] = {in_rfoot_pos(0), in_rfoot_pos(1), 0.0};  double rfoot_z = in_rfoot_pos(2);  
  m_seq_rfoot->set(&rfoot_xyth[0], in_wait_time);                 m_seq_rfoot_z->set(&rfoot_z, in_wait_time);


  // init left foot
  double lfoot_xyth[] = {in_lfoot_pos(0), in_lfoot_pos(1), 0.0};  double lfoot_z = in_lfoot_pos(2);
  m_seq_lfoot->set(&lfoot_xyth[0], in_wait_time);                 m_seq_lfoot_z->set(&lfoot_z, in_wait_time);


  // init zmp
  double zmp[] = {in_zmp(0), in_zmp(1), 0.0};
  m_seq_zmp->set(&zmp[0], in_wait_time);


  // init support leg
  double sup = (double)in_first_step;
  m_seq_sup->set(&sup, in_wait_time);



  
  //
  // for next step
  //
  if( in_first_step < 0 ) {
	m_ref_zmp = in_lfoot_pos + in_lfoot_rot * (m_to_lzmp - m_to_lfoot);
	m_ref_zmp(2) = 0.0;
  }
  else {
	m_ref_zmp = in_rfoot_pos + in_rfoot_rot * (m_to_rzmp - m_to_rfoot);
	m_ref_zmp(2) = 0.0;
  }
  
  // for stop
  m_pre_origin = in_zmp;


  m_end_is_stop = false;
  m_is_init = true;
}


void FootPlanner::set(double in_x, double in_y, double in_theta, bool enforced)
{
  if( !m_is_init )	return;


  static double timer(0.0);
	
  
  if( timer < 1.0e-6 || enforced ) {
	double pre_sup = *m_seq_sup->back();
	double nex_sup = -1 * pre_sup;

	Vector3 cur_pos(in_x, in_y, 0.0);  Matrix33 cur_rot(rodrigues(Vector3(0,0,1), in_theta));
	
	Vector3 ref_zmp(m_ref_zmp);
	Vector3 ref_swg;
	if( nex_sup < 0 ) {
	  m_ref_zmp = cur_pos + cur_rot * m_to_lzmp;   m_ref_zmp(2) = 0.0;  // for next step
	  ref_swg   = cur_pos + cur_rot * m_to_lfoot;  ref_swg(2) = in_theta;
	}
	else {
	  m_ref_zmp = cur_pos + cur_rot * m_to_rzmp;   m_ref_zmp(2) = 0.0;  // for next step
	  ref_swg   = cur_pos + cur_rot * m_to_rfoot;  ref_swg(2) = in_theta;
	}
	m_pre_origin = Vector3(in_x, in_y, in_theta);
	
	
	
	double step_time = m_t_zmp + m_t_wait + m_t_land + m_t_swg + m_t_land + m_t_wait;
	m_seq_zmp->set(&ref_zmp[0], m_t_zmp);  m_seq_zmp->set(&ref_zmp[0], (step_time-m_t_zmp));

	
	double ref_z(0.05);
	//double ref_z(0.0);
	if( nex_sup < 0 ) {
	  // support foot
	  m_seq_rfoot->set(0, step_time, 0.0, false);
	  m_seq_rfoot_z->set(0, step_time, 0.0, false);


	  // swing foot (x,y)
	  m_seq_lfoot->set(0, (m_t_zmp+m_t_wait+m_t_land), 0.0, false);
	  m_seq_lfoot->set(&ref_swg[0], m_t_swg);
	  m_seq_lfoot->set(0, (m_t_land+m_t_wait), 0.0, false);

	  // swing foot 9z)
	  m_seq_lfoot_z->set(0, (m_t_zmp+m_t_wait), 0.0, false);
	  m_seq_lfoot_z->set(&ref_z, (m_t_land+m_t_swg/2.0), 0.0, false);  // 上昇
	  ref_z *= -1;
	  m_seq_lfoot_z->set(&ref_z, (m_t_land+m_t_swg/2.0), 0.0, false);  // 下降
	  m_seq_lfoot_z->set(0, m_t_wait, 0.0, false);
	}
	else {
	  // swing foot (x,y)
	  m_seq_rfoot->set(0, (m_t_zmp+m_t_wait+m_t_land), 0.0, false);
	  m_seq_rfoot->set(&ref_swg[0], m_t_swg);
	  m_seq_rfoot->set(0, (m_t_land+m_t_wait), 0.0, false);

	  // swing foot 9z)
	  m_seq_rfoot_z->set(0, (m_t_zmp+m_t_wait), 0.0, false);
	  m_seq_rfoot_z->set(&ref_z, (m_t_land+m_t_swg/2.0), 0.0, false);  // 上昇
	  ref_z *= -1;
	  m_seq_rfoot_z->set(&ref_z, (m_t_land+m_t_swg/2.0), 0.0, false);  // 下降
	  m_seq_rfoot_z->set(0, m_t_wait, 0.0, false);


	  // support foot
	  m_seq_lfoot->set(0, step_time, 0.0, false);
	  m_seq_lfoot_z->set(0, step_time, 0.0, false);
	}


	m_seq_sup->set(&nex_sup, m_dt);  m_seq_sup->set(&nex_sup, (step_time-m_dt));
	timer = step_time;

	m_end_is_stop = false;
  }

  //test();

  timer -= m_dt;
}


void FootPlanner::stop()
{
  if( !m_is_init )	return;

  
  set(m_pre_origin(0), m_pre_origin(1), m_pre_origin(2), true);


  double pre_sup = *m_seq_sup->back();
  double nex_sup = -1 * pre_sup;
  
  Vector3 cur_pos(m_pre_origin(0), m_pre_origin(1), 0.0);  Matrix33 cur_rot(rodrigues(Vector3(0,0,1), m_pre_origin(2)));

  Vector3 ref_zmp(cur_pos);
  if( nex_sup < 0 ) {
	m_ref_zmp = cur_pos + cur_rot * m_to_lzmp;   m_ref_zmp(2) = 0.0;  // for next step
  }
  else {
	m_ref_zmp = cur_pos + cur_rot * m_to_rzmp;   m_ref_zmp(2) = 0.0;  // for next step
  }


  double t_stop(3.0);

  m_seq_zmp->set(&ref_zmp[0], m_t_zmp);  m_seq_zmp->set(&ref_zmp[0], (t_stop-m_t_zmp));

  m_seq_rfoot->set(0, t_stop, 0.0, false);
  m_seq_rfoot_z->set(0, t_stop, 0.0, false);

  m_seq_lfoot->set(0, t_stop, 0.0, false);
  m_seq_lfoot_z->set(0, t_stop, 0.0, false);

  m_seq_sup->set(&nex_sup, m_dt);  m_seq_sup->set(&nex_sup, (t_stop-m_dt));


  m_end_is_stop = true;
}


void FootPlanner::get(Vector3 &out_ref_zmp, Vector3 &out_ref_rfoot_pos, Matrix33 &out_ref_rfoot_rot, Vector3 &out_ref_lfoot_pos, Matrix33 &out_ref_lfoot_rot, Matrix33 &out_ref_waist_rot)
{ 
  m_seq_zmp->get(&out_ref_zmp[0]);


  Vector3 ref_rfoot_xyt;  double ref_rfoot_z;
  Vector3 ref_lfoot_xyt;  double ref_lfoot_z;

  m_seq_rfoot->get(&ref_rfoot_xyt[0]);  m_seq_rfoot_z->get(&ref_rfoot_z);
  m_seq_lfoot->get(&ref_lfoot_xyt[0]);  m_seq_lfoot_z->get(&ref_lfoot_z);

  out_ref_rfoot_pos = Vector3(ref_rfoot_xyt(0), ref_rfoot_xyt(1), ref_rfoot_z);
  out_ref_rfoot_rot = rodrigues(Vector3(0,0,1), ref_rfoot_xyt(2));

  out_ref_lfoot_pos = Vector3(ref_lfoot_xyt(0), ref_lfoot_xyt(1), ref_lfoot_z);
  out_ref_lfoot_rot = rodrigues(Vector3(0,0,1), ref_lfoot_xyt(2));

  out_ref_waist_rot = rodrigues(Vector3(0,0,1), (ref_rfoot_xyt(2)+ref_lfoot_xyt(2))/2.0);

  
  m_seq_sup->pop();
}


bool FootPlanner::empty()
{
  bool is_empty=false;

  if( m_seq_zmp->empty() )      is_empty = true;
  if( m_seq_rfoot->empty() )    is_empty = true;
  if( m_seq_rfoot_z->empty() )  is_empty = true;
  if( m_seq_lfoot->empty() )    is_empty = true;
  if( m_seq_lfoot_z->empty() )  is_empty = true;
  if( m_seq_sup->empty() )      is_empty = true;

  return is_empty;
}


std::deque<double*>* FootPlanner::zmpSequence()
{
  return m_seq_zmp->sequence();
}


FootType FootPlanner::supportFoot()
{
  double sup;
  m_seq_sup->get(&sup, false);

  if( sup < 0 )  return RFOOT;
  else           return LFOOT;
}


void FootPlanner::calcRodrigues(Matrix33& out_R, const Vector3& axis, double q)
{
  // E + a_hat*sin(q) + a_hat*a_hat*(1-cos(q))
  //
  //    |  0 -az  ay|
  // =E+| az   0 -ax|*s + a_hat*a_hat*v
  //    |-ay  ax   0|
  //
  //    |  0 -az  ay|     |-az*az-ay*ay        ax*ay        az*ax|
  // =E+| az   0 -ax|*s + |       ax*ay -az*az-ax*ax        ay*az|*v
  //    |-ay  ax   0|     |       az*ax        ay*az -ax*ax-ay*ay|
  //
  //  |1-az*az*v-ay*ay*v     -az*s+ax*ay*v      ay*s+az*ax*v|
  // =|     az*s+ax*ay*v 1-az*az*v-ax*ax*v     -ax*s+ay+az*v|
  //  |    -ay*s+az*ax*v      ax*s+ay*az*v 1-ax*ax*v-ay*ay*v|
  //
  
  const double sth = sin(q);
  const double vth = 1.0 - cos(q);
  
  double ax = axis(0);
  double ay = axis(1);
  double az = axis(2);
  
  const double axx = ax*ax*vth;
  const double ayy = ay*ay*vth;
  const double azz = az*az*vth;
  const double axy = ax*ay*vth;
  const double ayz = ay*az*vth;
  const double azx = az*ax*vth;
  
  ax *= sth;
  ay *= sth;
  az *= sth;
  
  out_R = 1.0 - azz - ayy, -az + axy,       ay + azx,
    az + axy,        1.0 - azz - axx, -ax + ayz,
    -ay + azx,       ax + ayz,        1.0 - ayy - axx;
}






void FootPlanner::test()
{
  std::cout
	<< "  " << m_seq_zmp->numSequence()
	<< "  " << m_seq_rfoot->numSequence()
	<< "  " << m_seq_rfoot_z->numSequence()
	<< "  " << m_seq_lfoot->numSequence()
	<< "  " << m_seq_lfoot_z->numSequence()
	<< "  " << m_seq_sup->numSequence()
	<< "\n\n" << std::endl;
}
