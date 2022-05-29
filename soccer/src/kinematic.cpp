#include "kinematic.hpp"
#include "algebra.hpp"

#include <Vector.h>
#include <Matrix.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

KinematicRobot::KinematicRobot(){
    X_OFFSET = 0;
    Y_OFFSET = 0;
    Z_OFFSET = 0;
    R_OFFSET = 0;
    P_OFFSET = 0;
    A_OFFSET = 0;
    HIP_PITCH_OFFSET = 0;

    PERIOD_TIME = 0;
    DSP_RATIO = 0;
    STEP_FB_RATIO = 0;
    Z_MOVE_AMPLITUDE = 0;
    Y_SWAP_AMPLITUDE = 0;
    Z_SWAP_AMPLITUDE = 0;
    ARM_SWING_GAIN = 0;

    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;    
    A_MOVE_AIM_ON = false;
}

void KinematicRobot::LoadJSON(std::string file_name) {
    std::string path = "data/" + file_name;
    std::ifstream file(path);
    nlohmann::json data = nlohmann::json::parse(file);
    for (auto &[key, val] : data.items()) {
        if (key == "ratio") {
            try{
                val.at("period_time").get_to(PERIOD_TIME);
                val.at("dsp_ratio").get_to(DSP_RATIO);
                val.at("step_forward_back_ratio").get_to(STEP_FB_RATIO);
                val.at("foot_height").get_to(Z_MOVE_AMPLITUDE);
                val.at("swing_right_left").get_to(Y_SWAP_AMPLITUDE);
                val.at("swing_top_down").get_to(Z_SWAP_AMPLITUDE);
                val.at("arm_swing_gain").get_to(ARM_SWING_GAIN);
            } catch (nlohmann::json::parse_error & ex) {
                std::cerr << "parse error at byte " << ex.byte << std::endl;
            }
        }
        else if (key == "init_angles") {
            try {
                val.at("right_shoulder_pitch").get_to(INIT_R_SHOULDER_PITCH);
                val.at("right_shoulder_roll").get_to(INIT_R_SHOULDER_ROLL);
                val.at("right_elbow").get_to(INIT_R_ELBOW);
                val.at("left_shoulder_pitch").get_to(INIT_L_SHOULDER_PITCH);
                val.at("left_shoulder_roll").get_to(INIT_L_SHOULDER_ROLL);
                val.at("left_elbow").get_to(INIT_L_ELBOW);
                val.at("right_hip_yaw").get_to(INIT_R_HIP_YAW);
                val.at("right_hip_roll").get_to(INIT_R_HIP_ROLL);
                val.at("right_hip_pitch").get_to(INIT_R_HIP_PITCH);
                val.at("right_knee").get_to(INIT_R_KNEE);
                val.at("right_ankle_pitch").get_to(INIT_R_ANKLE_PITCH);
                val.at("right_ankle_roll").get_to(INIT_R_ANKLE_ROLL);
                val.at("left_hip_yaw").get_to(INIT_L_HIP_YAW);
                val.at("left_hip_roll").get_to(INIT_L_HIP_ROLL);
                val.at("left_hip_pitch").get_to(INIT_L_HIP_PITCH);
                val.at("left_knee").get_to(INIT_L_KNEE);
                val.at("left_ankle_pitch").get_to(INIT_L_ANKLE_PITCH);
                val.at("left_ankle_roll").get_to(INIT_L_ANKLE_ROLL);
            } catch (nlohmann::json::parse_error& ex) {
                std::cerr << "parse error at byte " << ex.byte << std::endl;
            }
        }
        else if (key == "offset") {
            try {
                val.at("x_offset").get_to(X_OFFSET);
                val.at("y_offset").get_to(Y_OFFSET);
                val.at("z_offset").get_to(Z_OFFSET);
                val.at("yaw_offset").get_to(A_OFFSET);
                val.at("pitch_offset").get_to(P_OFFSET);
                val.at("roll_offset").get_to(R_OFFSET);
                val.at("hip_pitch_offset").get_to(HIP_PITCH_OFFSET);
            } catch (nlohmann::json::parse_error& ex) {
                std::cerr << "parse error at byte " << ex.byte << std::endl;
            }
        }
    }
}

void KinematicRobot::Initialize() {
    X_MOVE_AMPLITUDE   = 0;
    Y_MOVE_AMPLITUDE   = 0;
    A_MOVE_AMPLITUDE   = 0;

    m_X_Swap_Phase_Shift = alg::piValue();
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = alg::piValue() / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = alg::piValue() / 2;
    m_Z_Swap_Phase_Shift = alg::piValue() * 3 / 2;
    m_Z_Move_Phase_Shift = alg::piValue() / 2;
    m_A_Move_Phase_Shift = alg::piValue() / 2;

    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;
    update_param_time();
    update_param_move();

    Process();
}

double KinematicRobot::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
    return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

bool KinematicRobot::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
//   std::cout << "compute ik" << std::endl;
	Robot::Matrix3D Tad, Tda, Tcd, Tdc, Tac;
	Robot::Vector3D vec;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;

  // std::cout << "xyz: " << x << " " << y << " " << z << std::endl;
	Tad.SetTransform(Robot::Point3D(x, y, z - LEG_LENGTH), Robot::Vector3D(a * alg::rad2Deg(), b * alg::rad2Deg(), c * alg::rad2Deg()));

	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
  vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
  vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;
  // std::cout << "leg length: " << LEG_LENGTH << std::endl;
  // std::cout << "vec: " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;

  // Get Knee
	_Rac = vec.Length();
  _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
  if(std::isnan(_Acos) == 1){
    std::cout << "failed count knee value" << std::endl;
    return false;
  }
  *(out + 3) = _Acos;

  // Get Ankle Roll
  Tda = Tad;
	if(Tda.Inverse() == false){
    std::cout << "failed count ankle roll value" << std::endl;
    return false;
  }
    
  _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
  _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
  _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
  if(_m > 1.0)
    _m = 1.0;
  else if(_m < -1.0)
    _m = -1.0;
  _Acos = acos(_m);
  if(std::isnan(_Acos) == 1){
    std::cout << "failed count ankle roll 2 value" << std::endl;
    return false;
  }
  if(Tda.m[7] < 0.0)
    *(out + 5) = -_Acos;
  else
    *(out + 5) = _Acos;

  // Get Hip Yaw
	Tcd.SetTransform(Robot::Point3D(0, 0, -ANKLE_LENGTH), Robot::Vector3D(*(out + 5) * alg::rad2Deg(), 0, 0));
	Tdc = Tcd;
	if(Tdc.Inverse() == false){
    std::cout << "failed compute hip yaw value" << std::endl;
    return false;
  }
	Tac = Tad * Tdc;
  _Atan = atan2(-Tac.m[1] , Tac.m[5]);
  if(std::isinf(_Atan) == 1){
    std::cout << "failed compute hip value 2 value" << std::endl;
    return false;
  }
  *(out) = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
  if(std::isinf(_Atan) == 1){
    std::cout << "failed hip roll" << std::endl;
    return false;
  }
  *(out + 1) = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
  if(std::isinf(_Atan) == 1){
    std::cout << "failed hip pitch" << std::endl;
    return false;
  }
  _theta = _Atan;
  _k = sin(*(out + 3)) * CALF_LENGTH;
  _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
	_n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
  _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
  _c = (_n - _k * _s) / _l;
  _Atan = atan2(_s, _c);
  if(std::isinf(_Atan) == 1){
    std::cout << "failed ankle pitch" << std::endl;
    return false;
  }
  *(out + 2) = _Atan;
  *(out + 4) = _theta - *(out + 3) - *(out + 2);

  return true;
}

void KinematicRobot::update_param_time()
{
	m_PeriodTime = PERIOD_TIME;
  m_DSP_Ratio = DSP_RATIO;
  m_SSP_Ratio = 1 - m_DSP_Ratio;

  m_X_Swap_PeriodTime = m_PeriodTime / 2;
  m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
  m_Y_Swap_PeriodTime = m_PeriodTime;
  m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
  m_Z_Swap_PeriodTime = m_PeriodTime / 2;
  m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
  m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

  m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
  m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

  m_Phase_Time1 = (m_SSP_Time_Start_L + m_SSP_Time_End_L) / 2;
  m_Phase_Time2 = (m_SSP_Time_End_L + m_SSP_Time_Start_R) / 2;
  m_Phase_Time3 = (m_SSP_Time_Start_R + m_SSP_Time_End_R) / 2;

  m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void KinematicRobot::update_param_move()
{
  // Forward/Back
  m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
  m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

  // Right/Left
  m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
  if(m_Y_Move_Amplitude > 0)
      m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
  else
      m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
  m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

  m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
  m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
  m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
  m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

  // Direction
  if(A_MOVE_AIM_ON == false)
  {
      m_A_Move_Amplitude = A_MOVE_AMPLITUDE * alg::piValue() / 180.0 / 2;
      if(m_A_Move_Amplitude > 0)
          m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
      else
          m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
  }
  else
  {
      m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * alg::piValue() / 180.0 / 2;
      if(m_A_Move_Amplitude > 0)
          m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
      else
          m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
  }
}

void KinematicRobot::update_param_balance() {
  m_X_Offset = X_OFFSET;
  m_Y_Offset = Y_OFFSET;
  m_Z_Offset = Z_OFFSET;
  m_R_Offset = R_OFFSET * alg::piValue() / 180.0;
  m_P_Offset = P_OFFSET * alg::piValue() / 180.0;
  m_A_Offset = A_OFFSET * alg::piValue() / 180.0;
}


void KinematicRobot::Start()
{
    m_Ctrl_Running = true;
    m_Real_Running = true;
}

void KinematicRobot::Stop()
{
    m_Ctrl_Running = false;
}

void KinematicRobot::Process()
{
//   std::cout << "process walking" << std::endl;
  double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
  double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
  double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
  double angle[14], ep[12];
  double offset;
  double TIME_UNIT = 8;

  int dir[14] = {-1,-1,1,1,-1,1,-1,-1,-1,-1,1,1,1,-1};

  double initAngle[14];
  initAngle[0] = INIT_R_HIP_YAW;
  initAngle[1] = INIT_R_HIP_ROLL;
  initAngle[2] = INIT_R_HIP_PITCH;
  initAngle[3] = INIT_R_KNEE;
  initAngle[4] = INIT_R_ANKLE_PITCH;
  initAngle[5] = INIT_R_ANKLE_ROLL;
  initAngle[6] = INIT_L_HIP_YAW;
  initAngle[7] = INIT_L_HIP_ROLL;
  initAngle[8] = INIT_L_HIP_PITCH;
  initAngle[9] = INIT_L_KNEE;
  initAngle[10] = INIT_L_ANKLE_PITCH;
  initAngle[11] = INIT_L_ANKLE_ROLL;
  initAngle[12] = INIT_R_SHOULDER_PITCH;
  initAngle[13] = INIT_L_SHOULDER_PITCH;

  double outValue[14];

  if(m_Time == 0)
  {
      update_param_time();
      m_Phase = PHASE0;
      if(m_Ctrl_Running == false)
      {
          if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
          {
              m_Real_Running = false;
          }
          else
          {
              X_MOVE_AMPLITUDE = 0;
              Y_MOVE_AMPLITUDE = 0;
              A_MOVE_AMPLITUDE = 0;
          }
      }
  }
  else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
  {
      update_param_move();
      m_Phase = PHASE1;
  }
  else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
  {
      update_param_time();
      m_Time = m_Phase_Time2;
      m_Phase = PHASE2;
      if(m_Ctrl_Running == false)
      {
          if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
          {
              m_Real_Running = false;
          }
          else
          {
              X_MOVE_AMPLITUDE = 0;
              Y_MOVE_AMPLITUDE = 0;
              A_MOVE_AMPLITUDE = 0;
          }
      }
  }
  else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
  {
      update_param_move();
      m_Phase = PHASE3;
  }
  update_param_balance();

  x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
  y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
  z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
  a_swap = 0;
  b_swap = 0;
  c_swap = 0;

  if(m_Time <= m_SSP_Time_Start_L)
  {
      x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
      y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
      z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
      x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
      y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
      z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
  }
  else if(m_Time <= m_SSP_Time_End_L)
  {
      x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
      y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
      z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
      x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
      y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
      z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
  }
  else if(m_Time <= m_SSP_Time_Start_R)
  {
      x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
      y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
      z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
      x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
      y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
      z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
  }
  else if(m_Time <= m_SSP_Time_End_R)
  {
      x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
      y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
      z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
      x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
      y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
      z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
  }
  else
  {
      x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
      y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
      z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
      x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
      y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
      z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
      c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
  }

  a_move_l = 0;
  b_move_l = 0;
  a_move_r = 0;
  b_move_r = 0;

  ep[0] = x_swap + x_move_r + m_X_Offset;
  ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
  ep[2] = z_swap + z_move_r + m_Z_Offset;
  ep[3] = a_swap + a_move_r - m_R_Offset / 2;
  ep[4] = b_swap + b_move_r + m_P_Offset;
  ep[5] = c_swap + c_move_r - m_A_Offset / 2;
  ep[6] = x_swap + x_move_l + m_X_Offset;
  ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
  ep[8] = z_swap + z_move_l + m_Z_Offset;
  ep[9] = a_swap + a_move_l + m_R_Offset / 2;
  ep[10] = b_swap + b_move_l + m_P_Offset;
  ep[11] = c_swap + c_move_l + m_A_Offset / 2;

  // Compute arm swing
  if(m_X_Move_Amplitude == 0)
  {
      angle[12] = 0; // Right
      angle[13] = 0; // Left
  }
  else
  {
      angle[12] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
      angle[13] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
      angle[12] *= alg::deg2Rad();
      angle[13] *= alg::deg2Rad();
  }

  if(m_Real_Running == true)
  {
      m_Time += TIME_UNIT;
      if(m_Time >= m_PeriodTime)
          m_Time = 0;
  }

	// Compute angles
  if ((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  || (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false))
  {
    std::cout << "error inverse kinematics computation" << std::endl;
    return;
  }  

  for(int i=0; i<14; i++)
  {
    offset = (double)dir[i] * angle[i];
    if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
        offset -= (double)dir[i] * HIP_PITCH_OFFSET * alg::deg2Rad();

    outValue[i] = (initAngle[i]*alg::deg2Rad()) + offset;
  }

  joint_value[ID_R_HIP_YAW - 1] = outValue[0];
	joint_value[ID_R_HIP_ROLL - 1] = outValue[1];
	joint_value[ID_R_HIP_PITCH - 1] = outValue[2];
	joint_value[ID_R_KNEE - 1] = outValue[3];
	joint_value[ID_R_ANKLE_PITCH - 1] = outValue[4];
	joint_value[ID_R_ANKLE_ROLL - 1] = outValue[5];
	joint_value[ID_L_HIP_YAW - 1] = outValue[6];
	joint_value[ID_L_HIP_ROLL - 1] = outValue[7];
	joint_value[ID_L_HIP_PITCH - 1] = outValue[8];
	joint_value[ID_L_KNEE - 1] = outValue[9];
	joint_value[ID_L_ANKLE_PITCH - 1] = outValue[10];
	joint_value[ID_L_ANKLE_ROLL - 1] = outValue[11];
	joint_value[ID_R_SHOULDER_PITCH - 1] = outValue[12];
  joint_value[ID_R_SHOULDER_ROLL - 1] = INIT_R_SHOULDER_ROLL*alg::deg2Rad();
	joint_value[ID_R_ELBOW - 1] = INIT_R_ELBOW*alg::deg2Rad();
	joint_value[ID_L_SHOULDER_PITCH - 1] = outValue[13];
	joint_value[ID_L_SHOULDER_ROLL - 1] = INIT_L_SHOULDER_ROLL*alg::deg2Rad();
	joint_value[ID_L_ELBOW - 1] = INIT_L_ELBOW*alg::deg2Rad();
}