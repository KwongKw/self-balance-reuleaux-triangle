/**
MotorDriver 14, 12, 13, 27 (M1, M2, M3, EN)
AS5600      17, 16 (SDA, SCL)
MPU6050     19, 18 (SDA, SCL)
**/
#include <SimpleFOC.h>
#include "Kalman.h"  // Source: https://github.com/TKJElectronics/KalmanFilter
Kalman kalmanZ;
#define gyroZ_OFF -0.19
/* ----IMU Data---- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0;
uint32_t last_unstable_time;
uint32_t last_stable_time;
double gyroZangle;  // Angle calculate using the gyro only
double compAngleZ;  // Calculated angle using a complementary filter
double kalAngleZ;   // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data
/* ----FOC Data---- */

// driver instance
double acc2rotation(double x, double y);
float constrainAngle(float x);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
LowPassFilter lpf_throttle{ 0.00 };

//倒立摆参数
float LQR_K3_1 = 8.4;   //摇摆到平衡
float LQR_K3_2 = 1.7;   //
float LQR_K3_3 = 1.75;  //

float LQR_K4_1 = 2.4;   //平衡到稳定
float LQR_K4_2 = 1.5;   //
float LQR_K4_3 = 1.42;  //

//电机参数
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(14, 12, 13, 27);

float target_velocity = 0;   //目标速度
float target_angle = 89.3;   //平衡角度 例如TA89.3 设置平衡角度89.3
float target_voltage = 0;    //目标电压
float swing_up_voltage = 1;  //摇摆电压 左右摇摆的电压，越大越快到平衡态，但是过大会翻过头
float swing_up_angle = 10;   //摇摆角度 离平衡角度还有几度时候，切换到自平衡控制
float v_i_1 = 0.5;           //1      //非稳态速度环I
float v_p_1 = 0.05;          //0.1      //非稳态速度环P
float v_i_2 = 0.5;           //1      //稳态速度环I
float v_p_2 = 0.05;          //0.1      //稳态速度环P

void setup() {
  Serial.begin(115200);

  // kalman mpu6050 init
  Wire.begin(19, 18, 400000);  // Set I2C frequency to 400kHz
  i2cData[0] = 7;              // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;           // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;           // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;           // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false))
    ;  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ;  // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {  // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }
  delay(100);  // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = acc2rotation(accX, accY);
  kalmanZ.setAngle(pitch);
  gyroZangle = pitch;
  timer = micros();
  Serial.println("kalman mpu6050 init");

  I2Ctwo.begin(17, 16, 400000);  //SDA,SCL
  sensor.init(&I2Ctwo);

  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  //连接电机和driver对象
  motor.linkDriver(&driver);

  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //运动控制模式设置
  motor.controller = MotionControlType::velocity;
  //速度PI环设置
  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;

  //最大电机限制电机
  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 2;
  motor.current_limit = 1.5;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);

  //初始化电机
  motor.init();

  //初始化 FOC
  motor.initFOC();
}

char buf[255];
long loop_count = 0;
double last_pitch;
void loop() {
  motor.loopFOC();
  //    loop_count++ == 10
  //    loop_count = 0;
  while (i2cRead(0x3B, i2cData, 14))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();

  double pitch = acc2rotation(accX, accY);
  double gyroZrate = gyroZ / 131.0;  // Convert to deg/s
  if (abs(pitch - last_pitch) > 100)
    kalmanZ.setAngle(pitch);

  kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
  last_pitch = pitch;
  gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
  compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  float pendulum_angle = constrainAngle(fmod(kalAngleZ, 120) - target_angle);

  //   pendulum_angle当前角度与期望角度差值，在差值大的时候进行摇摆，差值小的时候LQR控制电机保持平衡
  if (abs(pendulum_angle) < swing_up_angle)  // if angle small enough stabilize 0.5~20°,1.5~90°
  {
    target_velocity = controllerLQR(pendulum_angle, gyroZrate, motor.shaft_velocity);
    if (abs(target_velocity) > 140)                    //120
      target_velocity = _sign(target_velocity) * 140;  //120
    motor.controller = MotionControlType::velocity;
    motor.move(target_velocity);
    // Serial.print(target_velocity);
  } else  // else do swing-up
  {       // sets swing_up_voltage to the motor in order to swing up
    motor.controller = MotionControlType::torque;
    target_voltage = -_sign(gyroZrate) * swing_up_voltage;
    motor.move(target_voltage);
  }

//Debug Console
#if 0
    Serial.print(target_velocity);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(kalAngleZ);
    Serial.print("\t");
    Serial.print(target_voltage);
    Serial.print("\t");
    Serial.print(motor.shaft_velocity);
    Serial.print("\t");
    Serial.print(motor.voltage.q);
    Serial.print("\t");
    Serial.print(target_angle);
    Serial.print("\t");
    Serial.print(pendulum_angle);
    Serial.print("\t");
    Serial.print(gyroZrate);
    Serial.print("\t");
    Serial.print("\r\n");
#endif
}
/* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
double acc2rotation(double x, double y) {
  double tmp_kalAngleZ = (atan(x / y) / 1.570796 * 90);
  if (y < 0) {
    return (tmp_kalAngleZ + 180);
  } else if (x < 0) {
    //将当前值与前值比较，当前差值大于100则认为异常
    if (!isnan(kalAngleZ) && (tmp_kalAngleZ + 360 - kalAngleZ) > 100) {
      //Serial.print("X<0"); Serial.print("\t");
      //Serial.print(tmp_kalAngleZ); Serial.print("\t");
      //Serial.print(kalAngleZ); Serial.print("\t");
      //Serial.print("\r\n");
      if (tmp_kalAngleZ < 0 && kalAngleZ < 0)  //按键右边角
        return tmp_kalAngleZ;
      else  //按键边异常处理
        return tmp_kalAngleZ;
    } else
      return (tmp_kalAngleZ + 360);
  } else {
    return tmp_kalAngleZ;
  }
}

// function constraining the angle in between -60~60
float constrainAngle(float x) {
  float a = 0;
  if (x < 0) {
    a = 120 + x;
    if (a < abs(x))
      return a;
  }
  return x;
}

// TODO PID stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum


// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel) {
  if (abs(p_angle) > 5)  //摆角大于5则进入非稳态，记录非稳态时间
  {
    last_unstable_time = millis();
    if (stable)  //如果是稳态进入非稳态则调整为目标角度
    {
      //target_angle = EEPROM.readFloat(0) - p_angle;
      stable = 0;
    }
  }
  if ((millis() - last_unstable_time) > 1000 && !stable)  //非稳态进入稳态超过500ms检测，更新目标角为目标角+摆角，假设进入稳态
  {
    //target_angle  -= _sign(target_velocity) * 0.4;
    target_angle = target_angle + p_angle;
    stable = 1;
  }

  if ((millis() - last_stable_time) > 2500 && stable) {  //稳态超过2000ms检测，更新目标角
    if (abs(target_velocity) > 5) {                      //稳态速度偏大校正
      last_stable_time = millis();
      target_angle -= _sign(target_velocity) * 0.2;
    }
  }

  float u;

  if (!stable)  //非稳态计算
  {
    motor.PID_velocity.P = v_p_1;
    motor.PID_velocity.I = v_i_1;
    u = LQR_K3_1 * p_angle + LQR_K3_2 * p_vel + LQR_K3_3 * m_vel;
  } else {
    motor.PID_velocity.P = v_p_2;
    motor.PID_velocity.I = v_i_2;
    u = LQR_K4_1 * p_angle + LQR_K4_2 * p_vel + LQR_K4_3 * m_vel;
  }

  return u;
}