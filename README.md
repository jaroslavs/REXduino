/****************************************************
 *                                                   *
 *  REXduino (REX master and Arduino slave)          *
 *                                                   *
 *  Jaroslav SOBOTA, 2013                            *
 *  email: jsobota@kky.zcu.cz                        * 
 *                                                   *
 ****************************************************/

The symbiosis of the REX Control System and an Arduino board is based on a simple communication protocol, whose slave part is implemented in the Arduino while the master part is running in the target device of the REX Control System (e.g. Raspberry Pi or your laptop or desktop PC running the RexCore runtime module). The master part is implemented using the programmable function block REXLANG. In fact, the Arduino serves only as an input/output unit.

Just compile and upload the REXduino_slave sketch to your Arduino board using the Arduino IDE (Wire and OneWire libraries are required for extending the functionality).

Open the appropriate sample project of REXduino master in RexDraw, compile it and download it to the target device of the REX Control System. 

The most basic projects for Arduino UNO are:
1) X_UNO_Blink_exec.mdl
2) X_UNO_PWMout_exec.mdl
3) X_UNO_Traffic_light_exec.mdl

In Simulink, start with the following example:
1) XSimulink_REXduino_UNO.mdl

You can also start from scratch by using one of the masks in REXduino_masks.mdl.

IMPORTANT!!! The file REXduino_master.stl must be present in the project folder, otherwise the project will not compile! 