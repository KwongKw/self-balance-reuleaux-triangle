# FYP_ESP32_Self-BalanceReuleauxTriangle
It is an FYP based on the design of the self-balance Reuleaux triangle. 

To maintain the unsatble equilibrium state. the force provided by the momentum wheel need to be calculated based on the physical state of the triangle.

![image](https://user-images.githubusercontent.com/56528924/181022367-afb7bdb0-2d54-45cd-841e-a076ff2b6504.png)

## Procedures

### PCB design

#### Apperance
  
- To ease the rotation of the system, the reuleaux triangle is used in the design, to minimize the force required for the system.

#### MCU

Arduino/ESP series MCUs are preferred. 
- Arduino has a well-developed library for implementing this project. 
ESP series provide wireless features including wireless programming, control, and system monitoring, and a relatively high compatibility with arduino library.

#### Sensors
- To monitor the state (angle of rotation) of the system, accelerometers are considered to be used to detect the tilt angle.

####	Motor
- To provide enough rotational force for the system, the momentum wheel is considered to be used to provide the required force

#### Charging Circuit (Optional)
- To prevent wires from affecting the mobility of the system. Power supply is better to be embedded into the system.

###	Control System
To enhance the performance for the system, different control systems will be tested on maintaining the unstable equilibrium system.

###	Calibratio
To tune the system, collect and interpret data extracted from the system, and make improvements on the system.
