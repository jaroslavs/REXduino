/****************************************************
 *                                                   *
 *  REXduino (REX master and Arduino slave)          *
 *                                                   *
 *  (c) Jaroslav SOBOTA, 2013                        *
 *  email: jsobota@kky.zcu.cz                        * 
 *                                                   *
 ****************************************************/

The symbiosis of the REX control system and an Arduino board is based on a simple communication protocol, whose slave part is implemented in the Arduino while the master part is running in the REX control system. The master part is implemented using the programmable function block REXLANG. In fact, the Arduino serves only as an input/output unit.

Just compile and upload the REXduino_slave sketch to your Arduino board using the Arduino IDE (Wire and OneWire libraries are required).

Open the appropriate sample project of REXduino master in RexDraw and create the control algorithm. Compile it and download it to the target device (e.g. Raspberry Pi or your laptop or desktop PC). 

You can also start from scratch by using one of the masks in REXduino_masks.mdl.