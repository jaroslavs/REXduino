
#REXduino (REXYGEN master and Arduino slave)#

*Last tested with:* [REXYGEN 2.50.10](http://www.rexygen.com) and [Arduino IDE 1.8.9](http://www.arduino.cc)

The symbiosis of REXYGEN and an Arduino board is based on a simple communication protocol. The slave part is implemented in the Arduino. In fact, the Arduino serves only as an input/output unit. The master part is running in the REXYGEN-enabled target device (e.g. Raspberry Pi or your laptop or desktop PC running the RexCore runtime module). The master part is implemented using the REXLANG programmable function block which is encapsulated inside a library subsystem. 

## Getting started with REXduino ##

1. Compile and upload the REXduino_slave sketch to your Arduino board using the Arduino IDE. 
    - OneWire library is required for DS18B20 temperature sensors to work *(last tested with version 2.3.5)*
    - Wire library is required for I2C devices 
2. Open the appropriate sample project of REXduino master in REXYGEN Studio, compile it and download it to the REXYGEN-enabled target device. 
3. The most basic projects for Arduino UNO are:
    - X_UNO_Blink_exec.mdl
    - X_UNO_PWMout_exec.mdl
    - X_UNO_Traffic_light_exec.mdl

You can also start from scratch by using one of the masks in REXduino_masks.mdl.

IMPORTANT!!! The file REXduino_master.stl must be present in the project folder, otherwise the project will not compile! 

----------
- Jaroslav SOBOTA
- 2013-2019
- e-mail: jsobota@ntis.zcu.cz 
