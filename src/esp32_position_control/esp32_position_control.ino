/**
 * ESP32 position motion control example with magnetic sensor
 */
#include <SimpleFOC.h>

// SPI Magnetic sensor instance (AS5047U example)
// MISO 12
// MOSI 9
// SCK 14
// magnetic sensor instance - SPI
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 15);

// I2C Magnetic sensor instance (AS5600 example)
// make sure to use the pull-ups!!
// SDA 21
// SCL 22
// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
//TwoWire I2Ctwo = TwoWire(1);
// Analog output Magnetic sensor instance (AS5600)
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(14, 12, 13, 27);


// angle set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  // link the motor to the sensor
  I2Ctwo.begin(17, 16, 400000);  //SDA,SCL
  sensor.init(&I2Ctwo);
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 0.05;
  // maximal voltage to be set to the motor
  //最大电机限制电机
  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 2;
  motor.current_limit = 1.5;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 40;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('M',doMotor,"motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();

  // user communication
  command.run();
}