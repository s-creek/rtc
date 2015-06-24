// -*- c++ -*-

#ifndef CREEK_FOOT_PLANNER_H
#define CREEK_FOOT_PLANNER_H

#include <iostream> // for debug
#include <deque>

#include <tvmet/Vector.h>
#include <tvmet/Matrix.h>


namespace creek
{
  class QueueInterpolator;
  enum FootType {RFOOT=-1, DFOOT, LFOOT};
};


namespace creek
{
  class FootPlanner
  {
  public:
    FootPlanner(double in_dt);
    ~FootPlanner();

	
	//
	//  |   shift zmp   |   wait   |   tale off foot   |   move swing foot   |   landing   |   wait   |
	//  |    (t_zmp)    | (t_wait) |      (t_land)     |       (t_swg)       |  (t_land)   | (t_wait) |
	//
	void setTimeParameter(double in_t_zmp, double in_t_wait, double in_t_land, double in_t_swg);
	void setFootParameter(double in_to_foot_x, double in_to_foot_y, double in_to_foot_z, double in_to_zmp_x, double in_to_zmp_y);
	void calcFootParameter(const tvmet::Vector<double, 3> &in_origin, const tvmet::Vector<double, 3> &in_rfoot_p, const tvmet::Vector<double, 3> &in_lfoot_p);

	
	void init(const tvmet::Vector<double, 3> &in_rfoot_pos, const tvmet::Matrix<double, 3, 3> &in_rfoot_rot,
			  const tvmet::Vector<double, 3> &in_lfoot_pos, const tvmet::Matrix<double, 3, 3> &in_lfoot_rot, const tvmet::Vector<double, 3> &in_zmp, double in_wait_time, creek::FootType in_first_step);
	void set(double in_x, double in_y, double in_theta, bool enforced=false);
	void stop();

	
	void get(tvmet::Vector<double, 3> &out_ref_zmp, tvmet::Vector<double, 3> &out_ref_rfoot_pos, tvmet::Matrix<double, 3, 3> &out_ref_rfoot_rot,
			 tvmet::Vector<double, 3> &out_ref_lfoot_pos, tvmet::Matrix<double, 3, 3> &out_ref_lfoot_rot, tvmet::Matrix<double, 3, 3> &out_ref_waist_rot);

	
	bool empty();
	inline bool isInit() { return m_is_init; }
	inline bool endIsStop() { return m_end_is_stop; }
	
	std::deque<double*> *zmpSequence();
	FootType supportFoot();
	

  private:
	//-----------------------------------------------------------------------
    // from OpenHRP
    //
	void calcRodrigues(tvmet::Matrix<double, 3, 3> &out_R, const tvmet::Vector<double, 3> &axis, double q);    
    inline tvmet::Matrix<double, 3, 3> rodrigues(const tvmet::Vector<double, 3> &axis, double q){
      tvmet::Matrix<double, 3, 3> R;
      calcRodrigues(R, axis, q);
      return R;
    }
    //-----------------------------------------------------------------------


	bool m_is_init, m_end_is_stop;
	tvmet::Vector<double, 3> m_ref_zmp, m_pre_origin;
	
	
	//
	// foot parameter
	//
	double m_dt;
	double m_t_zmp, m_t_wait, m_t_land, m_t_swg;

	tvmet::Vector<double, 3> m_to_rfoot, m_to_rzmp;
	tvmet::Vector<double, 3> m_to_lfoot, m_to_lzmp;


	//
	// ref seq
	//
	QueueInterpolator *m_seq_zmp;

	QueueInterpolator *m_seq_rfoot;    // (x, y, th)
	QueueInterpolator *m_seq_rfoot_z;  // (z)

	QueueInterpolator *m_seq_lfoot;    // (x, y, th)
	QueueInterpolator *m_seq_lfoot_z;  // (z)

	QueueInterpolator *m_seq_sup;


	void test();
  };
};

#endif
