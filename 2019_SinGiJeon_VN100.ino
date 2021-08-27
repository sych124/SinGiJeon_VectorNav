#include "VectorNav.h"
#include "variable.h"
#include "attitude_control.h"
#include "xbee_sd.h"

// SD, BMP, Serial3, VN100 Require Header file
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // BMP I2C
#define SD_CSpin 3 // SD pin

// Fin Servo PWM pin
#define FIN1_PIN 31
#define FIN2_PIN 35
#define FIN3_PIN 39
#define FIN4_PIN 43

int32_t LTime; //Launched Time
double bmp_height_before, bmp_velocity_before, bmp_accel_before;
long bmp_time_before, bmp_height_change_time, bmp_velocity_change_time;
double bmp_launch_site_height;
struct var_BMP bmploop; // height, velocity, accel from bmp 구조체
struct Init_att init_att;
float velx=0, vely = 0, velz = 0;
long last_time_integer = 0;

#define NARO_CENTER_PRESSURE 1013.25 // 기준기압, 기상청 참조

void(*resetFunc)(void) = 0;

void setup()
{
  /*Device Begin*/
  Serial.begin(57600);
  Serial3.begin(57600);    //Xbee comm
  IMU.begin();
  /*Fin control*/
  fin1_servo.attach(FIN1_PIN);
  fin2_servo.attach(FIN2_PIN);
  fin3_servo.attach(FIN3_PIN);
  fin4_servo.attach(FIN4_PIN);

  /*Device Check*/
  Serial.println();
  Serial3.println();
  Serial.println("Sequence Start.");
  Serial3.println("Sequence Start.");

  // Check 1. BMP
  Serial.print(F("1. BMP280 test... "));
  Serial3.print("1. BMP280 test... ");

  if (!bmp.begin())
  {
    Serial.println(F("BMP failed!"));
    Serial3.println("BMP failed!");
  }
  else
  {
    Serial.println("BMP OK");
    Serial3.println("BMP OK");
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Check 2. SD
  Serial.print("2. Initializing SD card...");
  Serial3.print("2. Initializing SD card...");

  if (!SD.begin(SD_CSpin))
  {
    Serial.println("initialization failed!");
    Serial3.println("initialization failed!");
  }
  else
  {
    Serial.println("initialization done. ");
    Serial3.println("initialization done. ");
  }

  // Check 3. SD file read/write
  TEXT_FILE_2019 = SD.open("2019_11.txt", FILE_WRITE);

  Serial.print("3. Text File Open... ");
  Serial3.print("3. Text File Open... ");

  if(TEXT_FILE_2019)
  {
    Serial.println("2019_11.txt opened.");
    Serial3.println("2019_11.txt opened.");
    TEXT_FILE_2019.println("2019. 11. 28. Launch Data");
    TEXT_FILE_2019.close();
  }
  else
  {
    Serial.println("2019_11.txt open failed.");
    Serial3.println("2019_11.txt open failed.");
  }

  Serial.println();

  // BMP initialize
  bmp_height_before = bmp_velocity_before = bmp_accel_before = 0; // double
  bmp_time_before = bmp_height_change_time = bmp_velocity_change_time = 0; // long

  // initialize
  bmp_launch_site_height = 0;
  Serial.println("BMP Initialize... ");
  Serial3.println("BMP Initialize... ");
  bmp_init();

  // IMU initialize  
  IMU.CMD_RESET;
  if (IMU.CMD_RESET)
  {
    Serial.println("IMU initialize OK");
    Serial3.println("IMU initialize OK");
  }
  else
  {
    Serial.println("IMU failed");
    Serial3.println("IMU failed");
  }

  // initial attitude
  struct NavLla Navlla;
  IMU.NavLlaPub(&IMU.NavLla);
  IMU.getEulerGyroAccel(&Navlla.yaw, &Navlla.pitch, &Navlla.roll, &Navlla.ax, &Navlla.ay, &Navlla.az, &Navlla.gx, &Navlla.gy, &Navlla.gz);
  
  init_att.roll = Navlla.roll;
  init_att.pitch = Navlla.pitch;
  init_att.yaw = Navlla.yaw;

  Serial.print("initial roll : "); Serial.print(Navlla.roll); Serial.println(" deg");
  Serial.print("initial pitch : "); Serial.print(Navlla.pitch); Serial.println(" deg");
  Serial.print("initial yaw : "); Serial.print(Navlla.yaw); Serial.println(" deg");
  Serial3.print("initial roll : "); Serial3.print(Navlla.roll); Serial3.println(" deg");
  Serial3.print("initial pitch : "); Serial3.print(Navlla.pitch); Serial3.println(" deg");
  Serial3.print("initial yaw : "); Serial3.print(Navlla.yaw); Serial3.println(" deg");

  Serial.println("IMU Initialize Success!");
  Serial.println();
  Serial3.println("IMU Initialize Success!");
  Serial3.println();
  
  if (Serial3.available())
  {
    Serial.println("XBee OK!");
  }
  else
  {
    Serial.println("XBee Re-check!");
  }

  while ('S' != Serial3.read())
  {
    if('A' == Serial3.read())
    {
      resetFunc(); 
    }
  }

  LTime = millis();
}

void loop()
{
  /*Sensor Collect*/
  struct FIN_CONTROL_DATA fin_data;
  struct var_BMP bmploop;
  struct NavLla navlla;
  long current_time_integer = millis();

  bmploop = loop_bmp_HVA();
  IMU.InsLlaPub(&IMU.InsLla);
  IMU.NavLlaPub(&IMU.NavLla);

  IMU.getEulerGyroAccel(&navlla.yaw, &navlla.pitch, &navlla.roll, &navlla.ax, &navlla.ay, &navlla.az, &navlla.gx, &navlla.gy, &navlla.gz);
  
  if(last_time_integer == 0)
    last_time_integer = current_time_integer;
  else{
    velx += navlla.ax*(current_time_integer - last_time_integer)/1000;
    vely += navlla.ay*(current_time_integer - last_time_integer)/1000;
    velz += navlla.az*(current_time_integer - last_time_integer)/1000;

    navlla.velx = velx;
    navlla.vely = vely;
    navlla.velz = velz;

    last_time_integer = current_time_integer;
    }

  /*Control*/
  fin_data = fin_control(navlla, init_att);

  /*Comm*/
  SD_write(navlla, fin_data, bmploop, LTime);
  Serial3_function(navlla, fin_data, bmploop, LTime);
//  Serial_function(navlla, fin_data, bmploop, LTime);
  if(Serial3.available())
  {
    Serial.println("Xbee OK");
  }

  if ('Q' == Serial3.read())
  {
    resetFunc();
  }
}

struct var_BMP loop_bmp_HVA(void)
{
  struct var_BMP HVA_result;
  double bmp_height_current = bmp.readAltitude(NARO_CENTER_PRESSURE); // current altitude
  long bmp_time_current = millis(); // current time

  double bmp_velocity_current = bmp_velocity(bmp_height_current, bmp_time_current);
  double bmp_accel_current = bmp_accel(bmp_velocity_current, bmp_time_current);

  bmp_velocity_before = bmp_velocity_current;
  bmp_height_before = bmp_height_current;
  bmp_time_before = bmp_time_current;
  bmp_accel_before = bmp_accel_current;

  HVA_result.h_BMP = bmp_height_before - bmp_launch_site_height;
  HVA_result.v_BMP = bmp_velocity_before;
  HVA_result.a_BMP = bmp_accel_before;

  return HVA_result;
}

double bmp_velocity(double bmp_height_current, long bmp_time_current)
{
  double dt = (double)bmp_time_current - (double)bmp_height_change_time;

  if(bmp_height_current == bmp_height_before)
  {
    return bmp_velocity_before;
  }
  else
  {
    bmp_height_change_time = bmp_time_current;
    return 1000 * (bmp_height_current - bmp_height_before) / dt;
  }
}

double bmp_accel(double bmp_velocity_current, long bmp_time_current)
{
  double dt = (double)bmp_time_current - (double)bmp_velocity_change_time;

  if(bmp_velocity_current == bmp_velocity_before)
  {
    return bmp_accel_before;
  }
  else
  {
    bmp_velocity_change_time = bmp_time_current;
    return 1000 * (bmp_velocity_current - bmp_velocity_before) / dt;
  }
}

void bmp_init(void)
{
  int i;

  for(i=0; i<10; i++)
  {
    double bmp_height_current = bmp.readAltitude(NARO_CENTER_PRESSURE); // current altitude
    long bmp_time_current = millis(); // current time

    double bmp_velocity_current = bmp_velocity(bmp_height_current, bmp_time_current);
    double bmp_accel_current = bmp_accel(bmp_velocity_current, bmp_time_current);

    Serial.print("H\t"); Serial.print(bmp_height_current); Serial.print("m\t");
    Serial.print("v\t"); Serial.print(bmp_velocity_current); Serial.print("m/s\t");
    Serial.print("a\t"); Serial.print(bmp_accel_current); Serial.print("m/s^2\n");
    Serial3.print("H\t"); Serial3.print(bmp_height_current); Serial3.print("m\t");
    Serial3.print("v\t"); Serial3.print(bmp_velocity_current); Serial3.print("m/s\t");
    Serial3.print("a\t"); Serial3.print(bmp_accel_current); Serial3.print("m/s^2\n");

    bmp_velocity_before = bmp_velocity_current;
    bmp_height_before = bmp_height_current;
    bmp_time_before = bmp_time_current;
    bmp_accel_before = bmp_accel_current;

    if(abs(bmp_accel_before) < 200)
    {
      int j;

      Serial.print("Launch Site Height Calculating... ");
      Serial3.print("Launch Site Height Calculating... ");

      for(j=0; j<5; j++)
      {
        bmp_launch_site_height += bmp.readAltitude(NARO_CENTER_PRESSURE);
        delay(600);
      }

      bmp_launch_site_height = bmp_launch_site_height / 5;
      Serial.print("Launch Site : "); Serial.print(bmp_launch_site_height); Serial.print(" m\n");
      Serial3.print("Launch Site : "); Serial3.print(bmp_launch_site_height); Serial3.print(" m\n");

      break;
    }

    delay(1000);
  }

  Serial.println("BMP Initialize Complete");
  Serial3.println("BMP Initialize Complete");
}
