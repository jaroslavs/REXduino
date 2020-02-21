# REXduino (REXYGEN master and Arduino slave) #

*Last tested with:* [REXYGEN 2.50.9](http://www.rexygen.com) and [Arduino IDE 1.8.12](http://www.arduino.cc)

The symbiosis of REXYGEN and an Arduino board is based on a simple communication protocol. The slave part is implemented in the Arduino. In fact, the Arduino serves only as an input/output unit. The master part is running in the REXYGEN-enabled target device (e.g. Raspberry Pi or your laptop or desktop PC running the RexCore runtime module). The master part is implemented using the REXLANG programmable function block which is encapsulated inside a library subsystem. 

## Getting started with REXduino ##

1. Compile and upload the REXduino_slave sketch to your Arduino board using the Arduino IDE. 
    - OneWire library is required for DS18B20 temperature sensors to work *(last tested with version 2.3.5)*
2. Open the appropriate sample project of REXduino master in REXYGEN Studio, compile it and download it to the REXYGEN-enabled target device. 
3. The most basic projects for Arduino UNO are:
    - 01_UNO_Blink_exec.mdl
    - 02_UNO_PWMout_exec.mdl
    - 05_UNO_Traffic_light_exec.mdl
4. There are wiring diagrams included in the project folders.

You can also start from scratch by using one of the blocks in REXduino_library.mdl.

## Using the REXduino library in REXYGEN ##

The files REXduino_library.mdl and REXduino_master.c must be placed in one of the locations below:

- the project folder
- the folder defined by the *LibraryPath* parameter of the PROJECT block included in the project
- the folder defined in the *Global library path* in REXYGEN Studio (*Settings->Program Options*)  

----------
- Jaroslav SOBOTA
- 2013-2020
- e-mail: jsobota@ntis.zcu.cz 
