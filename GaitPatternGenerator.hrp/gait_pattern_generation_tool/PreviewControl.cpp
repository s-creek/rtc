#include "PreviewControl.h"
#include <fstream>
#include <algorithm> // min()

using namespace std;
using namespace creek;

typedef tvmet::Matrix<double, 3, 3> Matrix33;
typedef tvmet::Matrix<double, 3, 2> Matrix32;
typedef tvmet::Vector<double, 3>    Vector3;
typedef tvmet::Vector<double, 2>    Vector2;


PreviewControl::PreviewControl()
{
  m_A_pre = tvmet::identity<Matrix33>();
  m_b_pre = 0.0;  m_c_pre = 0.0;
  
  m_preview_num = 0;

  m_pre_state = 0.0;  m_cur_state = 0.0;  m_ref_state = 0.0;
  
  m_uk = 0.0;

  m_is_load = false;
  m_is_init = false;
}


PreviewControl::PreviewControl(double in_dt, double in_com_height, double in_gravity)
{
  setParameter(in_dt, in_com_height, in_gravity);

  m_is_load = false;
  m_is_init = false;
}


void PreviewControl::setParameter(double in_dt, double in_com_height, double in_gravity)
{
  // set A
  m_A_pre = tvmet::identity<Matrix33>();
  m_A_pre(0,1) = in_dt;
  m_A_pre(0,2) = in_dt*in_dt/2.0;
  m_A_pre(1,2) = in_dt;

  // set b
  m_b_pre(0) = in_dt*in_dt*in_dt/6.0;
  m_b_pre(1) = in_dt*in_dt/2.0;
  m_b_pre(2) = in_dt;

  // set c
  m_c_pre(0) = 1.0;
  m_c_pre(1) = 0.0;
  m_c_pre(2) = -in_com_height/in_gravity;

  
  m_com_height = in_com_height;
}


bool PreviewControl::loadGain(string in_path_K, string in_path_f)
{
  m_is_load = false;
  
  
  ifstream file_K(in_path_K.c_str());
  if( !file_K )
    return false;

  file_K >> m_Ks;
  for(int i = 0; i < 3; i++)
    file_K >> m_Kx(i);

  file_K.close();

  

  ifstream file_f(in_path_f.c_str());
  if( !file_f )
    return false;

  m_fj.clear();
  m_preview_num = 0;
  string st;
  while(getline(file_f, st)) {
    if( st.size() > 1 ) {
      stringstream ss;
      ss.str(st);

      double tmp;
      ss >> tmp;
      m_fj.push_back(tmp);

      m_preview_num++;
    }
  }
  file_f.close();


  m_is_load = true;
  return true;
}


void PreviewControl::init(const Vector3 &in_com)
{
  m_is_init = false;
  if( !m_is_load )
	return;
  
  
  m_cur_state = 0.0;
  m_cur_state(0,0) = in_com(0);  m_cur_state(0,1) = in_com(1);  m_com_height = in_com(2);

  m_pre_state = m_cur_state;
  m_ref_state = m_cur_state;

  m_uk = 0.0;

  m_is_init = true;
}


void PreviewControl::get(const std::deque<double*> *in_ref_outputs, Vector3 &out_ref_com_pos, Vector3 &out_ref_com_vel, Vector3 &out_ref_com_acc, bool cycle)
{
  // calc p_k
  Vector2 output(tvmet::prod(tvmet::trans(m_cur_state), m_c_pre));


  // calc du_k
  int max_loop = min(m_preview_num, in_ref_outputs->size()) - 1;

  Vector2 sum_fj_zmp(0.0);
  for(int i = 0; i < max_loop; i++) {
    sum_fj_zmp(0) += ( m_fj[i] * (in_ref_outputs->at(i+1)[0] - in_ref_outputs->at(i)[0]) );
    sum_fj_zmp(1) += ( m_fj[i] * (in_ref_outputs->at(i+1)[1] - in_ref_outputs->at(i)[1]) );
  }

  Vector2 duk;
  Vector2 ref_outputs_front(in_ref_outputs->front()[0], in_ref_outputs->front()[1]);
  duk = -m_Ks * (output - ref_outputs_front) - tvmet::prod(tvmet::trans((m_cur_state - m_pre_state)), m_Kx) + sum_fj_zmp;


  if( cycle ) {
	m_uk += duk;
	m_ref_state = m_A_pre * m_cur_state + prod(m_b_pre, m_uk);

	m_pre_state = m_cur_state;
	m_cur_state = m_ref_state;
  }
  else {
	m_ref_state = m_A_pre * m_cur_state + prod(m_b_pre, Vector2(m_uk+duk));
  }


  out_ref_com_pos = Vector3(m_ref_state(0,0), m_ref_state(0,1), m_com_height);
  out_ref_com_vel = Vector3(m_ref_state(1,0), m_ref_state(1,1), 0.0);
  out_ref_com_acc = Vector3(m_ref_state(2,0), m_ref_state(2,1), 0.0);
}


Matrix32 PreviewControl::prod(const Vector3 &v3, const Vector2 &v2)
{
  Matrix32 out;
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 3; j++) {
      out(j,i) = v3(j) * v2(i);
    }
  }

  return out;
}
