struct FIN_CONTROL_DATA
{
   float require_Torque[3]; // roll, pitch, yaw
   int servo_delta[4];
};

typedef struct var_BMP{
  double h_BMP;
  double v_BMP;
  double a_BMP;
}VAR_BMP;

typedef struct Init_att{
  float yaw;
  float pitch;
  float roll;
};

typedef struct NavLla{
  float yaw;     // deg
  float pitch;
  float roll;
  float velx;    // m/s
  float vely;
  float velz;
  float ax;     // m/s^2
  float ay;
  float az;
  float gx;     // rad/s
  float gy;
  float gz;
};
