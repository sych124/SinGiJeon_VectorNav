#include <SD.h>
#include <SoftwareSerial.h>

File TEXT_FILE_2019; // sdcard file
// SoftwareSerial Serial3(15, 14); // Serial3 RX1, TX1(XBEE)

void SD_write(struct NavLla navlla, struct FIN_CONTROL_DATA fin_data, struct var_BMP bmploop, int32_t LTime)
{
	TEXT_FILE_2019 = SD.open("2019_11.txt", FILE_WRITE);
	TEXT_FILE_2019.print(millis()-LTime); TEXT_FILE_2019.print(" ms\t"); // 발사시점으로부터 경과 시간
	
	// BMP
  TEXT_FILE_2019.print(bmploop.h_BMP); TEXT_FILE_2019.print(" m\t"); // 도달 고도
  TEXT_FILE_2019.print(bmploop.v_BMP); TEXT_FILE_2019.print(" m/s\t");   // BMP vel
  TEXT_FILE_2019.print(bmploop.a_BMP); TEXT_FILE_2019.print(" m/s^2\t");  // BMP acc

  // accelerate
  TEXT_FILE_2019.print(navlla.ax); TEXT_FILE_2019.print(" m/s^2\t");   // X_accelerate
  TEXT_FILE_2019.print(navlla.ay); TEXT_FILE_2019.print(" m/s^2\t");   // Y_accelerate
  TEXT_FILE_2019.print(navlla.az); TEXT_FILE_2019.print(" m/s^2\t");   // Z_accelerate

  // attitude
  TEXT_FILE_2019.print(navlla.roll); TEXT_FILE_2019.print(" deg\t");   // roll_attitude
  TEXT_FILE_2019.print(navlla.pitch); TEXT_FILE_2019.print(" deg\t");   // pitch_attitude
  TEXT_FILE_2019.print(navlla.yaw); TEXT_FILE_2019.print(" deg\t");   // yaw_attitude

  // Require Torque
  TEXT_FILE_2019.print(fin_data.require_Torque[0]); TEXT_FILE_2019.print(" Nm\t"); // roll
  TEXT_FILE_2019.print(fin_data.require_Torque[1]); TEXT_FILE_2019.print(" Nm\t"); // pitch
  TEXT_FILE_2019.print(fin_data.require_Torque[2]); TEXT_FILE_2019.print(" Nm\t"); // yaw

  // Servo Delta output
  TEXT_FILE_2019.print(fin_data.servo_delta[0]); TEXT_FILE_2019.print("\t");
  TEXT_FILE_2019.print(fin_data.servo_delta[1]); TEXT_FILE_2019.print("\t");
  TEXT_FILE_2019.print(fin_data.servo_delta[2]); TEXT_FILE_2019.print("\t");
  TEXT_FILE_2019.print(fin_data.servo_delta[3]); TEXT_FILE_2019.println("\t");
  
	TEXT_FILE_2019.close();
}



void Serial3_function(struct NavLla navlla, struct FIN_CONTROL_DATA fin_data, struct var_BMP bmploop, int32_t LTime)
{
	Serial3.print(millis()-LTime); Serial3.print(" ms\t"); // 발사시점으로부터 경과 시간
	
	// BMP
	Serial3.print(bmploop.h_BMP); Serial3.print(" m\t"); // 도달 고도
  Serial3.print(bmploop.v_BMP); Serial3.print(" m/s\t");   // BMP vel
  Serial3.print(bmploop.a_BMP); Serial3.print(" m/s^2\t");  // BMP acc

  // accelerate
  Serial3.print(navlla.ax); Serial3.print(" m/s^2\t");   // X_accelerate
  Serial3.print(navlla.ay); Serial3.print(" m/s^2\t");   // Y_accelerate
  Serial3.print(navlla.az); Serial3.print(" m/s^2\t");   // Z_accelerate

  // Velcoity
  Serial3.print(navlla.velx); Serial3.print(" m/s\t");   // X_accelerate
  Serial3.print(navlla.vely); Serial3.print(" m/s\t");   // Y_accelerate
  Serial3.print(navlla.velz); Serial3.print(" m/s\t");   // Z_accelerate
  
  // attitude
  Serial3.print(navlla.roll); Serial3.print(" deg\t");   // roll_attitude
  Serial3.print(navlla.pitch); Serial3.print(" deg\t");   // pitch_attitude
  Serial3.print(navlla.yaw); Serial3.print(" deg\t");   // yaw_attitude

  // Require Torque
  Serial3.print(fin_data.require_Torque[0]); Serial3.print(" Nm\t"); // roll
  Serial3.print(fin_data.require_Torque[1]); Serial3.print(" Nm\t"); // pitch
  Serial3.print(fin_data.require_Torque[2]); Serial3.print(" Nm\t"); // yaw

  // Servo Delta output
  Serial3.print(fin_data.servo_delta[0]); Serial3.print("\t");
  Serial3.print(fin_data.servo_delta[1]); Serial3.print("\t");
  Serial3.print(fin_data.servo_delta[2]); Serial3.print("\t");
  Serial3.print(fin_data.servo_delta[3]); Serial3.println("\t");
}



void Serial_function(struct NavLla navlla, struct FIN_CONTROL_DATA fin_data, struct var_BMP bmploop, int32_t LTime)
{
  Serial.print(millis()-LTime); Serial.print(" ms\t"); // 발사시점으로부터 경과 시간
  
  // BMP
  Serial.print(bmploop.h_BMP); Serial.print(" m\t"); // 도달 고도
  Serial.print(bmploop.v_BMP); Serial.print(" m/s\t");   // BMP vel
  Serial.print(bmploop.a_BMP); Serial.print(" m/s^2\t");  // BMP acc

  // accelerate
  Serial.print(navlla.ax); Serial.print(" m/s^2\t");   // X_accelerate
  Serial.print(navlla.ay); Serial.print(" m/s^2\t");   // Y_accelerate
  Serial.print(navlla.az); Serial.print(" m/s^2\t");   // Z_accelerate

  // attitude
  Serial.print(navlla.roll); Serial.print(" deg\t");   // roll_attitude
  Serial.print(navlla.pitch); Serial.print(" deg\t");   // pitch_attitude
  Serial.print(navlla.yaw); Serial.print(" deg\t");   // yaw_attitude

  // Require Torque
  Serial.print(fin_data.require_Torque[0]); Serial.print(" Nm\t"); // roll
  Serial.print(fin_data.require_Torque[1]); Serial.print(" Nm\t"); // pitch
  Serial.print(fin_data.require_Torque[2]); Serial.print(" Nm\t"); // yaw

  // Servo Delta output
  Serial.print(fin_data.servo_delta[0]); Serial.print("\t");
  Serial.print(fin_data.servo_delta[1]); Serial.print("\t");
  Serial.print(fin_data.servo_delta[2]); Serial.print("\t");
  Serial.print(fin_data.servo_delta[3]); Serial.println("\t");
}
