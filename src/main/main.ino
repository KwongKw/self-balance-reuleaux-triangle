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

float LQR_K_1 = 7.1;   //70.8037
float LQR_K_2 = 1.7;   //17.1489
float LQR_K_3 = 1.75;     //1 ?

//电机参数
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(14, 12, 13, 27);

float target_velocity = 0;
float target_angle = 89.3;
float target_voltage = 0;
float swing_up_voltage = 1;
float swing_up_angle = 15;

float v_p = 0.2;   //0.05      //P
float v_i = 0.4;   //0.5      //I
float v_d = 0.0;  //0.01     //D
//---------------------------------------------------------------------------------------------------------------------//
void setup() {
  Serial.begin(115200);

  // kalman mpu6050 init
  Wire.begin(19, 18, 400000);  // Set I2C frequency to 400kHz
  i2cData[0] = 7;              // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;           // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;           // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;           // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false));  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));  // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {  // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
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

  // AS5600 init
  I2Ctwo.begin(17, 16, 400000);  //SDA,SCL
  sensor.init(&I2Ctwo);
  motor.linkSensor(&sensor);

  // FOCDriver init
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // control loop type and torque mode
  motor.controller = MotionControlType::velocity;

  // velocity loop PID
  motor.PID_velocity.P = v_p;
  motor.PID_velocity.I = v_i;
  motor.PID_velocity.D = v_d;
  motor.PID_velocity.limit = 40;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // Limits
  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 2;
  motor.current_limit = 1.5;

  //motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
}

double last_pitch;
//-----------------------------------------------------------------------------------------------------------------------------//
void loop() {
  motor.loopFOC();

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

  //   pendulum_angle = error
  if (abs(pendulum_angle) < swing_up_angle)  // if angle small enough stabilize
  {
    target_velocity = controllerLQR(pendulum_angle, gyroZrate, motor.shaft_velocity);
    if (abs(target_velocity) > 140)
      target_velocity = _sign(target_velocity) * 140;
    motor.controller = MotionControlType::velocity;
    motor.move(target_velocity);
  } else  // else do swing-up
  {       // sets swing_up_voltage to the motor in order to swing up
    motor.controller = MotionControlType::torque;
    target_voltage = -_sign(gyroZrate) * swing_up_voltage;
    motor.move(target_voltage);
  }

//Debug
#if 0
    Serial.print(target_velocity);Serial.print("\t");
    Serial.print(pitch);Serial.print("\t");
    Serial.print(kalAngleZ);Serial.print("\t");
    Serial.print(target_voltage);Serial.print("\t");
    Serial.print(motor.shaft_velocity);Serial.print("\t");
    Serial.print(motor.voltage.q);Serial.print("\t");
    Serial.print(target_angle);Serial.print("\t");
    Serial.print(pendulum_angle);Serial.print("\t");
    Serial.print(gyroZrate);Serial.print("\r\n");
#endif
}

double acc2rotation(double x, double y) {
  double tmp_kalAngleZ = atan(x / y) * 180.0 / PI;
  if (y < 0) {
    return (tmp_kalAngleZ + 180);
  } else if (x < 0) {
    return (tmp_kalAngleZ + 360);
  } else return tmp_kalAngleZ;
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

// TODO
// PID stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerPID(float p_angle){}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel) {
  if (abs(p_angle) > 5) {
    last_unstable_time = millis();
    stable = 0;
  }
  if ((millis() - last_unstable_time) > 1000 && !stable) {
    target_angle = target_angle + p_angle;
    stable = 1;
  }
  if ((millis() - last_stable_time) > 2500 && stable) {
    if (abs(target_velocity) > 5) {
      last_stable_time = millis();
      target_angle -= _sign(target_velocity) * 0.2;
    }
  }

  float u = LQR_K_1 * p_angle + LQR_K_2 * p_vel + LQR_K_3 * m_vel;
  return u;
}