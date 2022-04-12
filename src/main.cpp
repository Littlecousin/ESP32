
// #include <RF24.h>
#include <MPU6050_tockn.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "Arduino.h"
int V_feedback;
volatile long Velocity_L, Velocity_R;
int V_count, T_count;
float e_AngleY, e_Velocity, sum_e_Velocity, e_AngleZ, sum_e_AngleY, sum_e_GyroZ;
float B_Pwm, V_Pwm, T_Pwm, Pluse_L, Pluse_R, Angle_Turn_Setting, V_Setting;
float angleY, angleZ, gyroY, gyroZ;
long loop_timer;
unsigned long looptime, deltatime;
MPU6050 mpu6050(Wire);
// RF24 myRadio(4, 5); // NRF24L01 Pin(CE,CSN)
struct package
{
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};

byte addresses[][6] = {"0"};
typedef struct package Package;
Package data;

//左轮
#define MOTOR_A_PWM_PIN 33
#define MOTOR_A_PWM_CHANNEL 1
#define MOTOR_A_IN1_PIN (gpio_num_t)16
#define MOTOR_A_IN2_PIN (gpio_num_t)27
//右轮
#define MOTOR_B_PWM_PIN 25
#define MOTOR_B_PWM_CHANNEL 2
#define MOTOR_B_IN1_PIN (gpio_num_t)19  
#define MOTOR_B_IN2_PIN (gpio_num_t)17

/* Setting PWM Properties */
const int PWMFreq = 400; /* 400 Hz */
const int PWMResolution = 11;

//霍尔编码器测速引脚-左轮
#define LEFT_A_PIN 36
#define LEFT_B_PIN 39
#define LEFT_A digitalRead(LEFT_A_PIN)
#define LEFT_B digitalRead(LEFT_B_PIN)
//霍尔编码器测速引脚-右轮
#define RIGHT_A_PIN 35
#define RIGHT_B_PIN 34
#define RIGHT_A digitalRead(RIGHT_A_PIN)
#define RIGHT_B digitalRead(RIGHT_B_PIN)

TFT_eSPI tft = TFT_eSPI(); //设定屏幕

void Stop();
void moveForwardL();
void moveBackwardL();
void moveForwardR();
void moveBackwardR();
float Turn_PID_Angle();
float Velocity_PI();
float Blance_PD();
void counter0();
void counter1();
void counter2();
void counter3();
void reset();
void Encoder_Right_Handler();
void Encoder_Left_Handler();
void resetData()
{
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.pot1 = 0;
  data.pot2 = 0;
  data.tSwitch1 = 0;
  data.tSwitch2 = 0;
  data.button1 = 0;
  data.button2 = 0;
  data.button3 = 0;
  data.button4 = 0;
}

void setup()
{
  Serial.begin(115200);

  resetData();
  reset();

  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);

  pinMode(MOTOR_B_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);

  //霍尔编码器
  pinMode(LEFT_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_A_PIN), Encoder_Left_Handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A_PIN), Encoder_Right_Handler, CHANGE);

  // attachInterrupt(36, counter0, FALLING);           //interrupt pin36;
  // attachInterrupt(39, counter1, FALLING);           //interrupt pin39;

  // attachInterrupt(34, counter2, FALLING);           //interrupt pin34;
  // attachInterrupt(35, counter3, FALLING);           //interrupt pin35;

  ledcSetup(MOTOR_A_PWM_CHANNEL, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(MOTOR_A_PWM_PIN, MOTOR_A_PWM_CHANNEL);

  ledcSetup(MOTOR_B_PWM_CHANNEL, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channe2 to the GPIO Pin */
  ledcAttachPin(MOTOR_B_PWM_PIN, MOTOR_B_PWM_CHANNEL);

  // myRadio.begin();
  // myRadio.setChannel(115);
  // myRadio.setPALevel(RF24_PA_LOW);
  // myRadio.setDataRate(RF24_250KBPS);
  // myRadio.openReadingPipe(1, addresses[0]);
  // myRadio.startListening();

  tft.init();                             //初始化显示寄存器
  tft.fillScreen(TFT_WHITE);              //屏幕颜色
  tft.setTextColor(TFT_BLACK, TFT_WHITE); //设置字体颜色黑色
  tft.setCursor(15, 30, 1);               //设置文字开始坐标(15,30)及1号字体
  tft.setTextSize(2);
  tft.setTextFont(2);
  tft.setRotation(0);

  Wire.begin(21, 22);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true,500,500);
  // mpu6050.setGyroOffsets(-0.87, 1.03, 0.17);
}

void loop()
{
  
// Pluse_L = 512;
// Pluse_R = 512;


// moveForwardL();
// moveForwardR();
// delay(2000);
// moveBackwardL();
// moveBackwardR();
// delay(2000);



  //  deltatime = micros() - looptime;
  //  looptime = micros();
  //  Serial.println(deltatime);

  mpu6050.update();
  angleY = mpu6050.getAngleY(); // get angle and gyro value
  angleZ = mpu6050.getAngleZ();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  // tft.setCursor(0, 16 * 0 - 2);
  // tft.print("pluse_L:");
  // tft.setCursor(110, 16 * 0 - 2);
  // tft.println(Velocity_L);

  // tft.setCursor(0, 16 * 2 - 2);
  // tft.print("pluse_R:");
  // tft.setCursor(110, 16 * 2 - 2);
  // tft.println(Velocity_R);

  // tft.setCursor(0, 16 * 4 - 2);
  // tft.print("Motor1:");
  // tft.setCursor(110, 16 * 4 - 2);
  // tft.println(Motor1);

  // tft.setCursor(0, 16 * 6 - 2);
  // tft.print("Motor2:");
  // tft.setCursor(110, 16 * 6 - 2);
  // tft.println(Motor2);

  // tft.setCursor(0,16*8-0);
  // tft.print("AngleX:");
  // tft.setCursor(100,16*8-0);
  // tft.println(mpu6050.getAccAngleX());

  // tft.setCursor(0,16*10-0);
  // tft.print("AngleY:");
  tft.setCursor(100,16*10-0);
  tft.println(angleY);

  // tft.setCursor(0,16*12-0);
  // tft.print("AngleZ:");
  // tft.setCursor(100,16*12-0);
  // tft.println(angleZ);

  //Serial.println(angleY);

  // while (myRadio.available())
  // {
  //   myRadio.read(&data, sizeof(data));
  // }

  if (data.j2PotY > 180) // velovity setting(RPM)
  {
    V_Setting = map(data.pot2, 0, 255, 10, 100);
  }
  else if (data.j2PotY < 80)
  {
    V_Setting = map(data.pot2, 0, 255, -10, -100);
  }
  else // apply the brake
  {
    V_Setting = 0;

    if (sum_e_Velocity - 200 > 0)
    {
      sum_e_Velocity -= 200;
    }
    else if (sum_e_Velocity + 200 < 0)
    {
      sum_e_Velocity += 200;
    }
    else
    {
      sum_e_Velocity = 0;
    }
  }

  Angle_Turn_Setting = map(data.pot1, 0, 255, 270, -270); // angle of Z axis setting

  Blance_PD(); // blance pid runs each 5ms

  V_count++;
  if (V_count >= 4)
  {
    V_feedback = 60 * 100 * (Velocity_L + Velocity_R) / 1320; // RPM；

    Velocity_PI(); // velocity pid runs each 10ms

    Velocity_L = 0;
    Velocity_R = 0;
    V_count = 0;
  }

  T_count++;
  if (T_count >= 8)
  {
    Turn_PID_Angle(); // angle of Z axis pid runs each 20ms
    T_count = 0;
  }

  // Pluse_L = Blance_PD();
  // Pluse_R = Blance_PD();

  // Pluse_L = Velocity_PI();
  // Pluse_R = Velocity_PI();

  Pluse_L = Blance_PD() - Velocity_PI();// + Turn_PID_Angle(); // Pluse of left Motor caculate
  Pluse_R = Blance_PD() - Velocity_PI();// - Turn_PID_Angle(); // Pluse of left Motor caculate

  if (abs(angleY) < 65) // if angleY is between -65 and 65 ,run it
  {
    if ((Pluse_L) < 0)
    {
      moveForwardL();
    }
    else
    {
      moveBackwardL();
    }

    if ((Pluse_R) < 0)
    {
      moveBackwardR();
    }
    else
    {
      moveForwardR();
    }
  }
  else
  {
    reset();
    Stop();
  }

  while (micros() - loop_timer < 2000)
    ;                    // wait until 2000us are passed.
  loop_timer = micros(); // Set the timer for the next loop.
}

float med_angleY = -6.8;
static float Vertical_Sector = 0.6;
static float Vertical_Kp = 220*Vertical_Sector, Vertical_Kd = 5.2*Vertical_Sector;
static float Velocity_Kp = -0.25, Velocity_Ki = Velocity_Kp / 200;
float Blance_PD() // blance pid
{
  e_AngleY = angleY - med_angleY;
  B_Pwm = Vertical_Kp * e_AngleY + Vertical_Kd * gyroY;
  B_Pwm = constrain(B_Pwm, -2048, 2048);
  return B_Pwm;
}

float Velocity_PI() // velocity pid
{
  static float e_filter_out, e_filter_last;
  e_Velocity = V_feedback - V_Setting;
  e_filter_out = 0.7 * e_filter_last + 0.3 * e_Velocity; // Low-pass filter
  e_filter_last = e_filter_out;

  sum_e_Velocity += e_filter_out;

  sum_e_Velocity = constrain(sum_e_Velocity, -2048, 2048);
  V_Pwm = Velocity_Kp * e_filter_out + Velocity_Ki * sum_e_Velocity;

  V_Pwm = constrain(V_Pwm, -2048, 2048);

  return V_Pwm;
}
static float Turn_Kp = -5, Turn_Ki = -0.005, Turn_Kd = 0.4;
float Turn_PID_Angle() // angle of Z axis pid
{
  e_AngleZ = angleZ - Angle_Turn_Setting;
  sum_e_AngleY += e_AngleZ;
  T_Pwm = Turn_Kp * e_AngleZ + Turn_Ki * sum_e_AngleY + Turn_Kd * gyroZ;
  T_Pwm = constrain(T_Pwm, -2048, 2048);
  return T_Pwm;
}

void Encoder_Left_Handler()
{
  if (digitalRead(LEFT_A_PIN) == LOW)
  {
    if (digitalRead(LEFT_B_PIN) == LOW)
    {
      Velocity_L--; //反转
    }
    else if (digitalRead(LEFT_B_PIN) == HIGH)
    {
      Velocity_L++; //正转
    }
  }
  else if (digitalRead(LEFT_A_PIN) == HIGH)
  {
    if (digitalRead(LEFT_B_PIN) == LOW)
    {
      Velocity_L++; //正转
    }
    else if (digitalRead(LEFT_B_PIN) == HIGH)
    {
      Velocity_L--; //反转
    }
  }
}

void Encoder_Right_Handler()
{
  if (digitalRead(RIGHT_A_PIN) == LOW)
  {
    if (digitalRead(RIGHT_B_PIN) == LOW)
    {
      Velocity_R--; //反转
    }
    else if (digitalRead(RIGHT_B_PIN) == HIGH)
    {
      Velocity_R++; //正转
    }
  }
  else if (digitalRead(RIGHT_A_PIN) == HIGH)
  {
    if (digitalRead(RIGHT_B_PIN) == LOW)
    {
      Velocity_R++; //正转
    }
    else if (digitalRead(RIGHT_B_PIN) == HIGH)
    {
      Velocity_R--; //反转
    }
  }
}

void counter0() // left Encoder caculate
{
  if (digitalRead(39) == HIGH)
  {
    Velocity_L--;
  }
  if (digitalRead(39) == LOW)
  {
    Velocity_L++;
  }
}

void counter1()
{
  if (digitalRead(36) == HIGH)
  {
    Velocity_L++;
  }
  if (digitalRead(36) == LOW)
  {
    Velocity_L--;
  }
}

void counter2() // right Encoder caculate
{
  if (digitalRead(35) == HIGH)
  {
    Velocity_R++;
  }
  if (digitalRead(35) == LOW)
  {
    Velocity_R--;
  }
}
void counter3()
{
  if (digitalRead(34) == HIGH)
  {
    Velocity_R--;
  }
  if (digitalRead(34) == LOW)
  {
    Velocity_R++;
  }
}

void reset()
{
  sum_e_Velocity = 0;
  sum_e_AngleY = 0;
  sum_e_GyroZ = 0;
  B_Pwm = 0;
  V_Pwm = 0;
  T_Pwm = 0;
  T_count = 0;
  V_count = 0;
  Velocity_L = 0;
  Velocity_R = 0;
}

void moveForwardL() // left motor Forward；
{
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  ledcWrite(MOTOR_A_PWM_CHANNEL, abs(Pluse_L));
}
void moveBackwardL() // left motor Backward；
{
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(MOTOR_A_PWM_CHANNEL, abs(Pluse_L));
}

void moveForwardR() // right motor Forward；
{
  digitalWrite(MOTOR_B_IN1_PIN, HIGH);
  digitalWrite(MOTOR_B_IN2_PIN, LOW);
  ledcWrite(MOTOR_B_PWM_CHANNEL, abs(Pluse_R));
}
void moveBackwardR() // Right motor Backward；
{
  digitalWrite(MOTOR_B_IN1_PIN, LOW);
  digitalWrite(MOTOR_B_IN2_PIN, HIGH);
  ledcWrite(MOTOR_B_PWM_CHANNEL, abs(Pluse_R));
}

void Stop() // stop all；
{
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN1_PIN, LOW);
  digitalWrite(MOTOR_B_IN2_PIN, LOW);
}