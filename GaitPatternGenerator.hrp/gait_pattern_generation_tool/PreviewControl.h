// -*- c++ -*-

#ifndef CREEK_PREVIEW_CONTROL_H
#define CREEK_PREVIEW_CONTROL_H

#include <iostream>
#include <deque>

#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>

namespace creek
{
  class PreviewControl
  {
  public:
	PreviewControl();
	PreviewControl(double in_dt, double in_com_height, double in_gravity=9.8);
	~PreviewControl();

	void setParameter(double in_dt, double in_com_height, double in_gravity=9.8);
	bool loadGain(std::string in_path_K, std::string in_path_f);

	void init(const tvmet::Vector<double, 3> &in_com);
	void get(const std::deque<double*> *in_ref_outputs, tvmet::Vector<double, 3> &out_ref_com_pos, bool cycle=true) {
	  tvmet::Vector<double, 3> tmp_vel, tmp_acc;
	  get(in_ref_outputs, out_ref_com_pos, tmp_vel, tmp_acc, cycle);
	}
	void get(const std::deque<double*> *in_ref_outputs, tvmet::Vector<double, 3> &out_ref_com_pos, tvmet::Vector<double, 3> &out_ref_com_vel, tvmet::Vector<double, 3> &out_ref_com_acc, bool cycle=true);

	inline bool isInit() { return (m_is_init && m_is_load); }
	
	inline unsigned int numPreview() {
      return m_preview_num;
    }
	

  private:
	tvmet::Matrix<double, 3, 2> prod(const tvmet::Vector<double, 3> &v3, const tvmet::Vector<double, 2>  &v2);

	
	tvmet::Matrix<double, 3, 3> m_A_pre;
	tvmet::Vector<double, 3>    m_b_pre, m_c_pre;

	unsigned int m_preview_num;
	double m_com_height;
	
	double  m_Ks;
    tvmet::Vector<double, 3> m_Kx;
    std::deque<double> m_fj;

	tvmet::Matrix<double, 3, 2> m_pre_state, m_cur_state, m_ref_state;

	tvmet::Vector<double, 2> m_uk;

	bool m_is_load, m_is_init;
  };
};

#endif
