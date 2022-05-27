#include "walking.hpp"
#include "algebra.hpp"

#include <MotionStatus.h>
#include <Vector.h>
#include <Matrix.h>

#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>


WalkingRobot *WalkingRobot::m_UniqueInstance = nullptr;

WalkingRobot::WalkingRobot(webots::Robot* myRobot){
    mRobot = myRobot;
    mBasicTimeStep = myRobot->getBasicTimeStep();

    std::string motor_name[DGM_NMOTORS] = {
      "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
      "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
      "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
      "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
    };

    for (int i = 0; i < DGM_NMOTORS; i++)
        mMotors[i] = mRobot->getMotor(motor_name[i]);

    m_PeriodTime = 0;
    m_DSP_Ratio = 0;
    m_SSP_Ratio = 0;
    m_X_Swap_PeriodTime = 0;
    m_X_Move_PeriodTime = 0;
    m_Y_Swap_PeriodTime = 0;
    m_Y_Move_PeriodTime = 0;
    m_Z_Swap_PeriodTime = 0;
    m_Z_Move_PeriodTime = 0;
    m_A_Move_PeriodTime = 0;
    m_SSP_Time = 0;
    m_SSP_Time_Start_L = 0;
    m_SSP_Time_End_L = 0;
    m_SSP_Time_Start_R = 0;
    m_SSP_Time_End_R = 0;
    m_Phase_Time1 = 0;
    m_Phase_Time2 = 0;
    m_Phase_Time3 = 0;

    m_X_Offset = 0;
    m_Y_Offset = 0;
    m_Z_Offset = 0;
    m_R_Offset = 0;
    m_P_Offset = 0;
    m_A_Offset = 0;

    m_X_Move_Phase_Shift = 0;
    m_X_Move_Amplitude = 0;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Amplitude = 0;
    m_Y_Move_Phase_Shift = 0;
    m_Y_Move_Amplitude = 0;
    m_Y_Move_Amplitude_Shift = 0;
    m_Z_Swap_Amplitude = 0;
    m_Z_Move_Phase_Shift = 0;
    m_Z_Move_Amplitude = 0;
    m_Z_Move_Amplitude_Shift = 0;
    m_A_Move_Phase_Shift = 0;
    m_A_Move_Amplitude = 0;
    m_A_Move_Amplitude_Shift = 0;

    m_Pelvis_Offset = 0;
    m_Pelvis_Swing = 0;
    m_Arm_Swing_Gain = 0;

    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;
    A_MOVE_AIM_ON = false;

    BALANCE_ENABLE = true;

    HIP_COMP = 0.0;
    FOOT_COMP = 0.0;
}

void WalkingRobot::LoadJSON(std::string file_name) {
    std::string path = "data/motion/" + file_name;
    std::ifstream file(path);
    nlohmann::json data = nlohmann::json::parse(file);
    for (auto &[key, val] : data.items()) {
        if (key == "ratio") {
            try{
                val.at("period_time").get_to(PERIOD_TIME);
                val.at("dsp_ratio").get_to(DSP_RATIO);
                val.at("foot_height").get_to(Z_MOVE_AMPLITUDE);
                val.at("swing_right_left").get_to(Y_SWAP_AMPLITUDE);
                val.at("swing_up_down").get_to(Z_SWAP_AMPLITUDE);
                val.at("arm_swing_gain").get_to(ARM_SWING_GAIN);
                val.at("backward_hip_comp_ratio").get_to(BACKWARD_HIP_COMP_RATIO);
                val.at("forward_hip_comp_ratio").get_to(FORWARD_HIP_COMP_RATIO);
                val.at("foot_comp_ratio").get_to(FOOT_COMP_RATIO);
                val.at("dsp_comp_ratio").get_to(DSP_COMP_RATIO);
                val.at("period_comp_ratio").get_to(PERIOD_COMP_RATIO);
                val.at("move_accel_ratio").get_to(MOVE_ACCEL_RATIO);
                val.at("foot_accel_ratio").get_to(FOOT_ACCEL_RATIO);
            } catch (nlohmann::json::parse_error & ex) {
                std::cerr << "parse error at byte " << ex.byte << std::endl;
            }
        }
        else if (key == "balance") {
            try{
                val.at("enable").get_to(BALANCE_ENABLE);
                val.at("balance_knee_gain").get_to(BALANCE_KNEE_GAIN);
                val.at("balance_ankle_pitch_gain").get_to(BALANCE_ANKLE_PITCH_GAIN);
                val.at("balance_hip_roll_gain").get_to(BALANCE_HIP_ROLL_GAIN);
                val.at("balance_ankle_roll_gain").get_to(BALANCE_ANKLE_ROLL_GAIN);
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
        else if (key == "length") {
            try {
                val.at("thigh_length").get_to(THIGH_LENGTH);
                val.at("calf_length").get_to(CALF_LENGTH);
                val.at("ankle_length").get_to(ANKLE_LENGTH);
                val.at("leg_length").get_to(LEG_LENGTH);
            } catch (nlohmann::json::parse_error& ex) {
                std::cerr << "parse error at byte " << ex.byte << std::endl;
            }
        }
        else if (key == "angles_direction") {
            try {
                val.at("right_shoulder_pitch").get_to(DIR_R_SHOULDER_PITCH);
                val.at("right_hip_yaw").get_to(DIR_R_HIP_YAW);
                val.at("right_hip_roll").get_to(DIR_R_HIP_ROLL);
                val.at("right_hip_pitch").get_to(DIR_R_HIP_PITCH);
                val.at("right_knee").get_to(DIR_R_KNEE);
                val.at("right_ankle_pitch").get_to(DIR_R_ANKLE_PITCH);
                val.at("right_ankle_roll").get_to(DIR_R_ANKLE_ROLL);
                val.at("left_shoulder_pitch").get_to(DIR_L_SHOULDER_PITCH);
                val.at("left_hip_yaw").get_to(DIR_L_HIP_YAW);
                val.at("left_hip_roll").get_to(DIR_L_HIP_ROLL);
                val.at("left_hip_pitch").get_to(DIR_L_HIP_PITCH);
                val.at("left_knee").get_to(DIR_L_KNEE);
                val.at("left_ankle_pitch").get_to(DIR_L_ANKLE_PITCH);
                val.at("left_ankle_roll").get_to(DIR_L_ANKLE_ROLL);
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

void WalkingRobot::Initialize() {
    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;

    m_X_Move_Phase_Shift = alg::piValue() / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = alg::piValue() / 2;
    m_Z_Move_Phase_Shift = alg::piValue() / 2;
    m_A_Move_Phase_Shift = alg::piValue() / 2;

    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;

    update_param_time();
    update_param_move();

    Process();
}

void WalkingRobot::PrintAllVar() {
  std::cout << "Ratio:" << std::endl;
  std::cout << "PERIOD_TIME " << PERIOD_TIME << std::endl;
  std::cout << "DSP_RATIO " << DSP_RATIO << std::endl;
  std::cout << "Z_MOVE_AMPLITUDE " << Z_MOVE_AMPLITUDE << std::endl;
  std::cout << "Y_SWAP_AMPLITUDE " << Y_SWAP_AMPLITUDE << std::endl;
  std::cout << "Z_SWAP_AMPLITUDE " << Z_SWAP_AMPLITUDE << std::endl;
  std::cout << "ARM_SWING_GAIN " << ARM_SWING_GAIN << std::endl;
  std::cout << "BACKWARD_HIP_COMP_RATIO " << BACKWARD_HIP_COMP_RATIO << std::endl;
  std::cout << "FORWARD_HIP_COMP_RATIO " << FORWARD_HIP_COMP_RATIO << std::endl;
  std::cout << "FOOT_COMP_RATIO " << FOOT_COMP_RATIO << std::endl;
  std::cout << "DSP_COMP_RATIO " << DSP_COMP_RATIO << std::endl;
  std::cout << "PERIOD_COMP_RATIO " << PERIOD_COMP_RATIO << std::endl;
  std::cout << "MOVE_ACCEL_RATIO " << MOVE_ACCEL_RATIO << std::endl;
  std::cout << "FOOT_ACCEL_RATIO " << FOOT_ACCEL_RATIO << std::endl << std::endl;
  
  std::cout << "Balance:" << std::endl;
  std::cout << "BALANCE_KNEE_GAIN " << BALANCE_KNEE_GAIN << std::endl;
  std::cout << "BALANCE_ANKLE_PITCH_GAIN " << BALANCE_ANKLE_PITCH_GAIN << std::endl;
  std::cout << "BALANCE_HIP_ROLL_GAIN " << BALANCE_HIP_ROLL_GAIN << std::endl;
  std::cout << "BALANCE_ANKLE_ROLL_GAIN " << BALANCE_ANKLE_ROLL_GAIN << std::endl << std::endl;

  std::cout << "Init Angles:" << std::endl;
  std::cout << "INIT_R_SHOULDER_PITCH " << INIT_R_SHOULDER_PITCH << std::endl;
  std::cout << "INIT_R_SHOULDER_ROLL " << INIT_R_SHOULDER_ROLL << std::endl;
  std::cout << "INIT_R_ELBOW " << INIT_R_ELBOW << std::endl;
  std::cout << "INIT_L_SHOULDER_PITCH " << INIT_L_SHOULDER_PITCH << std::endl;
  std::cout << "INIT_L_SHOULDER_ROLL " << INIT_L_SHOULDER_ROLL << std::endl;
  std::cout << "INIT_L_ELBOW " << INIT_L_ELBOW << std::endl;
  std::cout << "INIT_R_HIP_YAW " << INIT_R_HIP_YAW << std::endl;
  std::cout << "INIT_R_HIP_ROLL " << INIT_R_HIP_ROLL << std::endl;
  std::cout << "INIT_R_HIP_PITCH " << INIT_R_HIP_PITCH << std::endl;
  std::cout << "INIT_R_KNEE " << INIT_R_KNEE << std::endl;
  std::cout << "INIT_R_ANKLE_PITCH " << INIT_R_ANKLE_PITCH << std::endl;
  std::cout << "INIT_R_ANKLE_ROLL " << INIT_R_ANKLE_ROLL << std::endl;
  std::cout << "INIT_L_HIP_YAW " << INIT_L_HIP_YAW << std::endl;
  std::cout << "INIT_L_HIP_ROLL " << INIT_L_HIP_ROLL << std::endl;
  std::cout << "INIT_L_HIP_PITCH " << INIT_L_HIP_PITCH << std::endl;
  std::cout << "INIT_L_KNEE " << INIT_L_KNEE << std::endl;
  std::cout << "INIT_L_ANKLE_PITCH " << INIT_L_ANKLE_PITCH << std::endl;
  std::cout << "INIT_L_ANKLE_ROLL " << INIT_L_ANKLE_ROLL << std::endl << std::endl;

  std::cout << "Kineamtic:" << std::endl;
  std::cout << "THIGH_LENGTH "<< THIGH_LENGTH << std::endl;
  std::cout << "CALF_LENGTH "<< CALF_LENGTH << std::endl;
  std::cout << "ANKLE_LENGTH "<< ANKLE_LENGTH << std::endl;
  std::cout << "LEG_LENGTH "<< LEG_LENGTH << std::endl << std::endl;

  std::cout << "Angle direction:" << std::endl;
  std::cout << "DIR_R_SHOULDER_PITCH " << DIR_R_SHOULDER_PITCH << std::endl;
  std::cout << "DIR_R_HIP_YAW " << DIR_R_HIP_YAW << std::endl;
  std::cout << "DIR_R_HIP_ROLL " << DIR_R_HIP_ROLL << std::endl;
  std::cout << "DIR_R_HIP_PITCH " << DIR_R_HIP_PITCH << std::endl;
  std::cout << "DIR_R_KNEE " << DIR_R_KNEE << std::endl;
  std::cout << "DIR_R_ANKLE_PITCH " << DIR_R_ANKLE_PITCH << std::endl;
  std::cout << "DIR_R_ANKLE_ROLL " << DIR_R_ANKLE_ROLL << std::endl;
  std::cout << "DIR_L_SHOULDER_PITCH " << DIR_L_SHOULDER_PITCH << std::endl;
  std::cout << "DIR_L_HIP_YAW " << DIR_L_HIP_YAW << std::endl;
  std::cout << "DIR_L_HIP_ROLL " << DIR_L_HIP_ROLL << std::endl;
  std::cout << "DIR_L_HIP_PITCH " << DIR_L_HIP_PITCH << std::endl;
  std::cout << "DIR_L_KNEE " << DIR_L_KNEE << std::endl;
  std::cout << "DIR_L_ANKLE_PITCH " << DIR_L_ANKLE_PITCH << std::endl;
  std::cout << "DIR_L_ANKLE_ROLL " << DIR_L_ANKLE_ROLL << std::endl << std::endl;

  std::cout << "Offset:" << std::endl;
  std::cout << "X_OFFSET " << X_OFFSET << std::endl;
  std::cout << "Y_OFFSET " << Y_OFFSET << std::endl;
  std::cout << "Z_OFFSET " << Z_OFFSET << std::endl;
  std::cout << "A_OFFSET " << A_OFFSET << std::endl;
  std::cout << "P_OFFSET " << P_OFFSET << std::endl;
  std::cout << "R_OFFSET " << R_OFFSET << std::endl;
}

double WalkingRobot::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
    return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

bool WalkingRobot::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
  std::cout << "compute ik" << std::endl;
	Matrix3D Tad, Tda, Tcd, Tdc, Tac;
	Vector3D vec;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;

  std::cout << "xyz: " << x << " " << y << " " << z << std::endl;
	Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * alg::rad2Deg(), b * alg::rad2Deg(), c * alg::rad2Deg()));

	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
  vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
  vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;
  std::cout << "leg length: " << LEG_LENGTH << std::endl;
  std::cout << "vec: " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;

  // Get Knee
	_Rac = vec.Length();
  _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
  if(isnan(_Acos) == 1){
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
  if(isnan(_Acos) == 1){
    std::cout << "failed count ankle roll 2 value" << std::endl;
    return false;
  }
  if(Tda.m[7] < 0.0)
    *(out + 5) = -_Acos;
  else
    *(out + 5) = _Acos;

  // Get Hip Yaw
	Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * alg::rad2Deg(), 0, 0));
	Tdc = Tcd;
	if(Tdc.Inverse() == false){
    std::cout << "failed compute hip yaw value" << std::endl;
    return false;
  }
	Tac = Tad * Tdc;
  _Atan = atan2(-Tac.m[1] , Tac.m[5]);
  if(isinf(_Atan) == 1){
    std::cout << "failed compute hip value 2 value" << std::endl;
    return false;
  }
  *(out) = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
  if(isinf(_Atan) == 1){
    std::cout << "failed hip roll" << std::endl;
    return false;
  }
  *(out + 1) = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
  if(isinf(_Atan) == 1){
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
  if(isinf(_Atan) == 1){
    std::cout << "failed ankle pitch" << std::endl;
    return false;
  }
  *(out + 2) = _Atan;
  *(out + 4) = _theta - *(out + 3) - *(out + 2);

  return true;
}

void WalkingRobot::update_param_time()
{
  DSP_COMP = fabs(m_X_Move_Amplitude) * DSP_COMP_RATIO * 0.001;

	m_PeriodTime = PERIOD_TIME - (fabs(m_X_Move_Amplitude) * PERIOD_COMP_RATIO);

  m_DSP_Ratio = DSP_RATIO + DSP_COMP;
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

  if (m_X_Move_Amplitude > 0)
    HIP_COMP = m_X_Move_Amplitude * FORWARD_HIP_COMP_RATIO;
  else
    HIP_COMP = m_X_Move_Amplitude * BACKWARD_HIP_COMP_RATIO;

  FOOT_COMP = fabs(m_X_Move_Amplitude) * FOOT_COMP_RATIO;
}

void WalkingRobot::update_param_move()
{
  // Forward/Back
  double x_input = X_MOVE_AMPLITUDE;
  double y_input = Y_MOVE_AMPLITUDE * 0.5;
  double a_input = A_MOVE_AMPLITUDE;

  if (m_Z_Move_Amplitude < (Z_MOVE_AMPLITUDE * 0.45))
  {
    x_input = 0.0;
    y_input = 0.0;
    a_input = 0.0;
  }

  m_X_Move_Amplitude = alg::smoothValue(m_X_Move_Amplitude, x_input, MOVE_ACCEL_RATIO);
  m_Y_Move_Amplitude = alg::smoothValue(m_Y_Move_Amplitude, y_input, MOVE_ACCEL_RATIO);
  m_Y_Move_Amplitude_Shift = fabs(m_Y_Move_Amplitude);
  m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

  if (m_Ctrl_Running)
    m_Z_Move_Amplitude = alg::smoothValue(m_Z_Move_Amplitude, (Z_MOVE_AMPLITUDE + FOOT_COMP) * 0.5, FOOT_ACCEL_RATIO);
  else
    m_Z_Move_Amplitude = 0.0;

  m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude;
  m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;

  if (A_MOVE_AIM_ON == false)
  {
    m_A_Move_Amplitude = alg::smoothValue(m_A_Move_Amplitude, a_input * alg::deg2Rad() * 0.5, MOVE_ACCEL_RATIO);
    m_A_Move_Amplitude_Shift = fabs(m_A_Move_Amplitude);
  }
  else
  {
    m_A_Move_Amplitude = alg::smoothValue(m_A_Move_Amplitude, (-a_input) * alg::deg2Rad() * 0.5, MOVE_ACCEL_RATIO);
    m_A_Move_Amplitude_Shift = -fabs(m_A_Move_Amplitude);
  }
}


void WalkingRobot::Start()
{
    m_Ctrl_Running = true;
    m_Real_Running = true;
}

void WalkingRobot::Stop()
{
    m_Ctrl_Running = false;
    while (IsRunning())
        step(8);
}

void WalkingRobot::Process()
{
  std::cout << "process walking" << std::endl;
  // Update walk parameters
	double TIME_UNIT = 8;

  if (m_Time == 0)
  {
    update_param_move();
    update_param_time();

    if (m_Ctrl_Running == false)
    {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_X_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_Y_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_A_Move_Amplitude) <= 5.0);

      m_Z_Move_Amplitude = 0;

      if (walk_in_position)
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
  else if (m_Time >= (m_Phase_Time1 - TIME_UNIT / 2) && m_Time < (m_Phase_Time1 + TIME_UNIT / 2))
  {
    update_param_move();
  }
  else if (m_Time >= (m_Phase_Time2 - TIME_UNIT / 2) && m_Time < (m_Phase_Time2 + TIME_UNIT / 2))
  {
    update_param_move();
    update_param_time();
    m_Time = m_Phase_Time2;

    if(m_Ctrl_Running == false)
    {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_X_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_Y_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_A_Move_Amplitude) <= 5.0);

      m_Z_Move_Amplitude = 0;

      if (walk_in_position)
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
  else if (m_Time >= (m_Phase_Time3 - TIME_UNIT / 2) && m_Time < (m_Phase_Time3 + TIME_UNIT / 2))
  {
    update_param_move();
  }

  m_X_Offset = X_OFFSET;
  m_Y_Offset = Y_OFFSET;
  m_Z_Offset = Z_OFFSET;
  m_R_Offset = R_OFFSET * alg::deg2Rad();
  m_P_Offset = P_OFFSET * alg::deg2Rad();
  m_A_Offset = A_OFFSET * alg::deg2Rad();

  // Compute endpoints
  double x_swap = 0;
  double y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, 0, m_Y_Swap_Amplitude, 0);
  double z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, alg::piValue() * 1.5, m_Z_Swap_Amplitude, 0);
  double a_swap = 0;
  double b_swap = 0;
  double c_swap = 0;

  double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
  double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;

  a_move_l = 0;
  b_move_l = 0;
  a_move_r = 0;
  b_move_r = 0;
  
  if (m_Time <= m_SSP_Time_Start_L)
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
  else if (m_Time <= m_SSP_Time_End_L)
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
  else if (m_Time <= m_SSP_Time_Start_R)
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
  else if (m_Time <= m_SSP_Time_End_R)
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

  double r_x = x_swap + x_move_r + m_X_Offset;
  double r_y = y_swap + y_move_r - m_Y_Offset / 2;
  double r_z = z_swap + z_move_r + m_Z_Offset;
  double r_a = a_swap + a_move_r - m_R_Offset / 2;
  double r_b = b_swap + b_move_r + m_P_Offset;
  double r_c = c_swap + c_move_r - m_A_Offset / 2;

  double l_x = x_swap + x_move_l + m_X_Offset;
  double l_y = y_swap + y_move_l + m_Y_Offset / 2;
  double l_z = z_swap + z_move_l + m_Z_Offset;
  double l_a = a_swap + a_move_l + m_R_Offset / 2;
  double l_b = b_swap + b_move_l + m_P_Offset;
  double l_c = c_swap + c_move_l + m_A_Offset / 2;

  double angle[14];
  for (int i = 0; i < 14; i++)
  {
    angle[i] = 0;
  }

  if(m_X_Move_Amplitude == 0)
  {
    angle[13] = 0; // Right
    angle[14] = 0; // Left
  }
  else
  {
    angle[13] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    angle[14] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
  }

  if (m_Real_Running == true)
  {
    m_Time += TIME_UNIT;
    if (m_Time >= m_PeriodTime)
    {
      m_Time = 0;
    }
  }
  else
  {
    m_Time = 0;
  }

	// Compute angles
  if ((computeIK(&angle[0], r_x, r_y, r_z, r_a, r_b, r_c) == false)
  || (computeIK(&angle[6], l_x, l_y, l_z, l_a, l_b, l_c) == false))
  {
    std::cout << "error inverse kinematics computation" << std::endl;
    return;
  }

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

  int dir[14];
  dir[0] = DIR_R_HIP_YAW;
  dir[1] = DIR_R_HIP_ROLL;
  dir[2] = DIR_R_HIP_PITCH;
  dir[3] = DIR_R_KNEE;
  dir[4] = DIR_R_ANKLE_PITCH;
  dir[5] = DIR_R_ANKLE_ROLL;
  dir[6] = DIR_L_HIP_YAW;
  dir[7] = DIR_L_HIP_ROLL;
  dir[8] = DIR_L_HIP_PITCH;
  dir[9] = DIR_L_KNEE;
  dir[10] = DIR_L_ANKLE_PITCH;
  dir[11] = DIR_L_ANKLE_ROLL;
  dir[12] = DIR_R_SHOULDER_PITCH;
  dir[13] = DIR_L_SHOULDER_PITCH;

  double outValue[14];
  for (int i = 0; i < 14; i++)
  {
    outValue[i] = (double)dir[i] * angle[i] + alg::deg2Rad()*initAngle[i];
    if (i == 2 || i == 8)
      outValue[i] += HIP_PITCH_OFFSET*alg::deg2Rad();
  }

  // adjust balance offset
  if (BALANCE_ENABLE == true)
  {
    double rlGyroErr = MotionStatus::RL_GYRO;
    double fbGyroErr = MotionStatus::FB_GYRO;

    outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4);
    outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4);

    outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN * 4);
    outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN * 4);

    outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4);
    outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4);

    outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4);
    outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4);
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

void WalkingRobot::step(int step) {
    if (step < 8) {
        std::cerr << "walking robot: steps of less than 8ms are not supported" << std::endl;
        return;
    }

    int numberOfStepToProcess = step / 8;
    if (BALANCE_ENABLE && (mRobot->getGyro("Gyro")->getSamplingPeriod() <= 0)) {
        std::cerr << "gyro is not enabled" << std::endl;
        mRobot->getGyro("Gyro")->enable(mBasicTimeStep);
        myStep();
    }

    for (int i = 0; i < numberOfStepToProcess; i++) {
        if (BALANCE_ENABLE) {
            const double* gyro = mRobot->getGyro("Gyro")->getValues();
            Robot::MotionStatus::RL_GYRO = gyro[0] - 512;
            Robot::MotionStatus::FB_GYRO = gyro[1] - 512;
        }
        Process();
    }

    for (int i = 0; i < (DGM_NMOTORS - 2); i++) {
      mMotors[i]->setPosition(getJointValue(i));
      std::cout << i + 1 << ":" << getJointValue(i)*alg::rad2Deg() << ", ";
    }
    std::cout << std::endl;    
}

void WalkingRobot::myStep() {
    int ret = mRobot->step(mBasicTimeStep);
    if (ret == -1)
        exit(EXIT_SUCCESS);
}