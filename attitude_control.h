#include <math.h>
#include <Servo.h>

Publisher IMU((uint8_t)22);  // CSpin

#define Kp_r 0.3
#define Ki_r 0.001
#define Kd_r 0.01 // roll control PID coefficient

#define Kp 0.1
#define Ki 0.01
#define Kd 0.001 // pitch, yaw control PID conefficient

int32_t past_time = 0;

// Fin Servo PWM pin
#define FIN1_PIN 31
#define FIN2_PIN 35
#define FIN3_PIN 39
#define FIN4_PIN 43

Servo fin1_servo;
Servo fin2_servo;
Servo fin3_servo;
Servo fin4_servo;

double CM_table[16][10] = { // table[Angle of Attack][delta], AoA as 0:1:15, delta as -25:5:20
  { 1.0488,  1.0022,  0.8509 ,  0.5616,  0.2700,  0.0000, -0.2700, -0.5616 , -0.8509, -1.0022},  
  { 0.7332,  0.6885,  0.5439 ,  0.2687, -0.0051, -0.2729, -0.5593, -0.8675 , -1.1705, -1.3283}, 
  { 0.4263,  0.3835,  0.2457 , -0.0129, -0.2797, -0.5604, -0.8650, -1.1884 , -1.5044, -1.6682},
  { 0.1289,  0.0885, -0.0409 , -0.2894, -0.5646, -0.8644, -1.1845, -1.5226 , -1.8509, -2.0214},
  {-0.1566, -0.1948, -0.3195 , -0.5716, -0.8654, -1.1814, -1.5167, -1.8682 , -2.2094, -2.3831},
  {-0.4353, -0.4732, -0.5999 , -0.8675, -1.1786, -1.5101, -1.8593, -2.2249 , -2.5722, -2.7436},
  {-0.7113, -0.7509, -0.8855 , -1.1699, -1.4960, -1.8416, -2.2042, -2.5802 , -2.9236, -3.0890},
  {-0.9994, -1.0418, -1.1850 , -1.4836, -1.8245, -2.1834, -2.5592, -2.9357 , -3.2684, -3.4287},  
  {-1.2991, -1.3438, -1.4941 , -1.8065, -2.1610, -2.5338, -2.9156, -3.2840 , -3.6068, -3.7637},
  {-1.6074, -1.6542, -1.8114 , -2.1370, -2.5048, -2.8884, -3.2670, -3.6240 , -3.9400, -4.0918},
  {-1.9234, -1.9723, -2.1361 , -2.4738, -2.8551, -3.2409, -3.6097, -3.9588 , -4.2657, -4.4117},
  {-2.2463, -2.2971, -2.4669 , -2.8173, -3.2065, -3.5867, -3.9458, -4.2876 , -4.5836, -4.7243},
  {-2.5749, -2.6274, -2.8035 , -3.1651, -3.5540, -3.9244, -4.2772, -4.6087 , -4.8945, -5.0279},
  {-2.9089, -2.9635, -3.1458 , -3.5122, -3.8943, -4.2569, -4.6022, -4.9230 , -5.1958, -5.3186},
  {-3.2486, -3.3047, -3.4904 , -3.8548, -4.2279, -4.5849, -4.9202, -5.2303 , -5.4841, -5.5932},
  {-3.5908, -3.6475, -3.8330 , -4.1906, -4.5571, -4.9067, -5.2322, -5.5276 , -5.7565, -5.8508}
};

int find_index(double *ary, double target);
int delta_result[4];
struct FIN_CONTROL_DATA delta_calculate(int AoA, double torque[3], struct NavLla navlla);
struct FIN_CONTROL_DATA fin_control(struct NavLla navlla, struct Init_att init_att);

float last_attitude_yaw = 0;
float last_attitude_pitch = 0;
float last_attitude_roll = 0;


float roll_i=0, pitch_i=0, yaw_i=0; // Integrate

struct FIN_CONTROL_DATA delta_calculate(int AoA, float torque[3], struct NavLla navlla) // torque : [roll, pitch, yaw]
{
  int i; // each pin
  struct FIN_CONTROL_DATA fin;
  float that = navlla.velx * navlla.velx + navlla.vely * navlla.vely + navlla.velz * navlla.velz;
  
  fin.require_Torque[0] = torque[0];
  fin.require_Torque[1] = torque[1];
  fin.require_Torque[2] = torque[2];

  if((torque[0] > torque[1]) && (torque[0] > torque[2])) // roll 요구 Torque가 제일 큼
  { // T = 0.0021*V*V*C_M -> C_M = T / (0.0021*V*V)
    float roll_CM = torque[0]/(0.0021 * that);
	  float roll_delta = find_index(CM_table[AoA], roll_CM) * 5 - 25;

	  for(i=0; i<4; i++)
	  {
	    fin.servo_delta[i] = (int)roll_delta;
	  }
  }
  else // pitch, yaw control
  { // T = 0.0092*V*V*C_M -> C_M = T / (0.0092*V*V)
    float yaw_CM = torque[2]/(0.0092 * that);
    float pitch_CM = torque[1]/(0.0092 * that);

    float yaw_delta = find_index(CM_table[AoA], yaw_CM) * 5 - 25;
    float pitch_delta = find_index(CM_table[AoA], pitch_CM) * 5 - 25;

	  fin.servo_delta[0] = fin.servo_delta[2] = (int)yaw_delta;
	  fin.servo_delta[1] = fin.servo_delta[3] = (int)pitch_delta;
  }

  return fin;
}

int find_index(double* ary, double target)
{
  int mindiff_index = 0;
  double mindiff_value = abs(ary[0] - target);
  int i;

  for(i=1; i<10; i++)
  {
    if(abs(ary[i] - target) < mindiff_value)
    {
      mindiff_value = abs(ary[i] - target);
      mindiff_index = i;
    }
  }

  return mindiff_index;
}

struct FIN_CONTROL_DATA fin_control(struct NavLla navlla, struct Init_att init_att)
{
	int AoA;
	float yaw_error, pitch_error, roll_error;
  float yaw_w, pitch_w, roll_w;
  struct FIN_CONTROL_DATA fin_control_data;

  yaw_error = navlla.yaw - init_att.yaw;
  pitch_error = navlla.pitch - init_att.pitch;
  roll_error = navlla.roll - init_att.roll;

  yaw_w = (navlla.yaw - last_attitude_yaw) / ((millis() - past_time) / 1000.);
  pitch_w = (navlla.pitch - last_attitude_pitch) / ((millis() - past_time) / 1000.);
  roll_w = (navlla.roll - last_attitude_roll) / ((millis() - past_time) / 1000.);

  AoA = (int) 180*atan(navlla.velz/navlla.velx)/3.1415;
  
	yaw_i += yaw_error * ((millis() - past_time) / 1000.); // angle integrate, [degree * sec]
	pitch_i += pitch_error * ((millis() - past_time) / 1000.); // angle integrate, [degree * sec]
	roll_i += roll_error * ((millis() - past_time) / 1000.); // angle integrate, [degree * sec]

				                            // angle 			          angle Integrate		Angular Velocity
				                            // P Gain			          Integrate Gain		Derivative Gain
	fin_control_data.require_Torque[2] = Kp * yaw_error    +	Ki * (yaw_i)    +	Kd * (yaw_w); // yaw
	fin_control_data.require_Torque[1] = Kp * pitch_error  +	Ki * (pitch_i)  +	Kd * (pitch_w); // pitch
	fin_control_data.require_Torque[0] = Kp_r * roll_error +	Ki_r * (roll_i) +	Kd_r * (roll_w); // roll

	fin_control_data = delta_calculate(AoA, fin_control_data.require_Torque, navlla);

  fin1_servo.write(fin_control_data.servo_delta[0]);
	fin2_servo.write(fin_control_data.servo_delta[1]);
	fin3_servo.write(fin_control_data.servo_delta[2]);
	fin4_servo.write(fin_control_data.servo_delta[3]);

	past_time = millis();

  last_attitude_yaw = navlla.yaw;
  last_attitude_pitch = navlla.pitch;
  last_attitude_roll = navlla.roll;

  return fin_control_data;
}
