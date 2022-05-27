#ifndef WALKING_ROBOT_HPP
#define WALKING_ROBOT_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <MotionModule.h>
#include <string>

#define DGM_NMOTORS 20

using namespace Robot;

class WalkingRobot {
    public:
		enum
		{
			ID_R_SHOULDER_PITCH     = 1,
			ID_L_SHOULDER_PITCH     = 2,
			ID_R_SHOULDER_ROLL      = 3,
			ID_L_SHOULDER_ROLL      = 4,
			ID_R_ELBOW              = 5,
			ID_L_ELBOW              = 6,
			ID_R_HIP_YAW            = 7,
			ID_L_HIP_YAW            = 8,
			ID_R_HIP_ROLL           = 9,
			ID_L_HIP_ROLL           = 10,
			ID_R_HIP_PITCH          = 11,
			ID_L_HIP_PITCH          = 12,
			ID_R_KNEE               = 13,
			ID_L_KNEE               = 14,
			ID_R_ANKLE_PITCH        = 15,
			ID_L_ANKLE_PITCH        = 16,
			ID_R_ANKLE_ROLL         = 17,
			ID_L_ANKLE_ROLL         = 18,
			ID_HEAD_PAN             = 19,
			ID_HEAD_TILT            = 20,
			NUMBER_OF_JOINTS
		};

		enum
		{
			PHASE0 = 0,
			PHASE1 = 1,
			PHASE2 = 2,
			PHASE3 = 3
		};

		WalkingRobot(webots::Robot* myRobot);
		~WalkingRobot() {}

		void LoadJSON(std::string file_name);
		void Initialize();
		void Start();
		void Stop();
		
		void Process();
  		bool IsRunning() { return m_Real_Running; }
		void step(int step);
		double getJointValue(int i){ return joint_value[i]; }

        // Walking initial pose
		double X_OFFSET;
		double Y_OFFSET;
		double Z_OFFSET;
		double A_OFFSET;
		double P_OFFSET;
		double R_OFFSET;

		// Walking control
		double PERIOD_TIME;
		double DSP_RATIO;
		double STEP_FB_RATIO;
		double X_MOVE_AMPLITUDE;
		double Y_MOVE_AMPLITUDE;
		double Z_MOVE_AMPLITUDE;
		double A_MOVE_AMPLITUDE;
		bool A_MOVE_AIM_ON;

		// Balance control
		bool   BALANCE_ENABLE;
		double BALANCE_KNEE_GAIN;
		double BALANCE_ANKLE_PITCH_GAIN;
		double BALANCE_HIP_ROLL_GAIN;
		double BALANCE_ANKLE_ROLL_GAIN;
		double Y_SWAP_AMPLITUDE;
		double Z_SWAP_AMPLITUDE;
		double ARM_SWING_GAIN;
		double HIP_PITCH_OFFSET;

		// Init angles
		double INIT_R_HIP_YAW;
		double INIT_R_HIP_ROLL;
		double INIT_R_HIP_PITCH;
		double INIT_R_KNEE;
		double INIT_R_ANKLE_PITCH;
		double INIT_R_ANKLE_ROLL;
		double INIT_R_SHOULDER_PITCH;
		double INIT_R_SHOULDER_ROLL;
		double INIT_R_ELBOW;

		double INIT_L_HIP_YAW;
		double INIT_L_HIP_ROLL;
		double INIT_L_HIP_PITCH;
		double INIT_L_KNEE;
		double INIT_L_ANKLE_PITCH;
		double INIT_L_ANKLE_ROLL;
		double INIT_L_SHOULDER_PITCH;
		double INIT_L_SHOULDER_ROLL;
		double INIT_L_ELBOW;

		// dir angles
		double DIR_R_HIP_YAW;
		double DIR_R_HIP_ROLL;
		double DIR_R_HIP_PITCH;
		double DIR_R_KNEE;
		double DIR_R_ANKLE_PITCH;
		double DIR_R_ANKLE_ROLL;
		double DIR_R_SHOULDER_PITCH;

		double DIR_L_HIP_YAW;
		double DIR_L_HIP_ROLL;
		double DIR_L_HIP_PITCH;
		double DIR_L_KNEE;
		double DIR_L_ANKLE_PITCH;
		double DIR_L_ANKLE_ROLL;
		double DIR_L_SHOULDER_PITCH;

		// Kineamtic
		double THIGH_LENGTH;
		double CALF_LENGTH;
		double ANKLE_LENGTH;
		double LEG_LENGTH;

    private:
		double wsin(double time, double period, double period_shift, double mag, double mag_shift);
		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
		void update_param_time();
		void update_param_move();
		void update_param_balance();
		void myStep();

		webots::Robot *myRobot;
        int timeStep;

		webots::Motor* motor[20];
		double joint_value[20];

		double m_PeriodTime;
		double m_DSP_Ratio;
		double m_SSP_Ratio;
		double m_X_Swap_PeriodTime;
		double m_X_Move_PeriodTime;
		double m_Y_Swap_PeriodTime;
		double m_Y_Move_PeriodTime;
		double m_Z_Swap_PeriodTime;
		double m_Z_Move_PeriodTime;
		double m_A_Move_PeriodTime;
		double m_SSP_Time;
		double m_SSP_Time_Start_L;
		double m_SSP_Time_End_L;
		double m_SSP_Time_Start_R;
		double m_SSP_Time_End_R;
		double m_Phase_Time1;
		double m_Phase_Time2;
		double m_Phase_Time3;

		double m_X_Offset;
		double m_Y_Offset;
		double m_Z_Offset;
		double m_R_Offset;
		double m_P_Offset;
		double m_A_Offset;

		double m_X_Swap_Phase_Shift;
		double m_X_Swap_Amplitude;
		double m_X_Swap_Amplitude_Shift;
		double m_X_Move_Phase_Shift;
		double m_X_Move_Amplitude;
		double m_X_Move_Amplitude_Shift;
		double m_Y_Swap_Phase_Shift;
		double m_Y_Swap_Amplitude;
		double m_Y_Swap_Amplitude_Shift;
		double m_Y_Move_Phase_Shift;
		double m_Y_Move_Amplitude;
		double m_Y_Move_Amplitude_Shift;
		double m_Z_Swap_Phase_Shift;
		double m_Z_Swap_Amplitude;
		double m_Z_Swap_Amplitude_Shift;
		double m_Z_Move_Phase_Shift;
		double m_Z_Move_Amplitude;
		double m_Z_Move_Amplitude_Shift;
		double m_A_Move_Phase_Shift;
		double m_A_Move_Amplitude;
		double m_A_Move_Amplitude_Shift;

		double m_Arm_Swing_Gain;

		bool m_Ctrl_Running;
		bool m_Real_Running;
		double m_Time;

		int    m_Phase;
		double m_Body_Swing_Y;
		double m_Body_Swing_Z;
};

#endif