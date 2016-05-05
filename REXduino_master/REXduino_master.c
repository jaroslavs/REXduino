/****************************************************
*                                                   *
*  REXduino master                                  *
*  REX and Arduino communication over serial port   *
*                                                   *
*  Jaroslav SOBOTA, 2016                            *
*  email: jsobota@kky.zcu.cz                        *
*                                                   *
****************************************************/

/*************************************************************************
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**************************************************************************/

#define MAJORVERSION 1
#define MINORVERSION 3 //even number = release, odd number = development
#define REVISION 0 //for hotfixes, even number = hotfix applied, odd number = development
#define COMMIT 0 //

#define COM_BAUDRATE 57600 //change this line according to desired baudrate
#define COMMAND_LENGTH_MAX 8 //maximum length of command 
#define RESPONSE_BUFFER_LENGTH 20 //buffer for responses
#define COMM_INIT_INTERVAL 2000 //period of communication re-initialization [ms]
#define COM_REOPEN_INTERVAL 5000 //period of re-opening the serial port [ms]
#define TIMEOUT_FAC 5 //timeout factor (timeout = TIMEOUT_FAC * task_period, task period is given by EXEC parameters)

#define MASKTYPE_ARD_UNO              1 //Arduino UNO
#define MASKTYPE_ARD_UNOhex           2 //Arduino UNO hexadecimal mask
#define MASKTYPE_ARD_MEGA2560         3 //Arduino MEGA 2560
#define MASKTYPE_ARD_MEGA2560hex      4 //Arduino MEGA 2560 hexadecimal mask
#define MASKTYPE_SEEED_MEGAV122       5 //Seeeduino Mega v1.22
#define MASKTYPE_SEEED_MEGAV122hex    6 //Seeeduino Mega v1.22 hexadecimal mask

#define STATUS_ONEWIRE_NOMOREDEVICES  50 //2 - must be the same as in REXduino.ino
#define STATUS_ONEWIRE_TEMPCONV       52 //4 - must be the same as in REXduino.ino 

#define ERROR_OPENING_PORT 14648 //err_code 57 (ASCII 9), err_subcode 56 (ASCII 8)
#define ERROR_RESPONSE_INVALID 14649 //err_code 57 (ASCII 9), err_subcode 57 (ASCII 9)
#define ERROR_ONEWIRE             52 // 1-Wire-related error response from Arduino 
#define ERROR_ONEWIRE_BUSYBUS       50 // 1-Wire bus busy

#define CON_COM1 33
#define CON_COM2 34
#define CON_COM3 35
#define CON_COM4 36
#define CON_COM5 37
#define CON_COM6 38
#define CON_COM7 39
#define CON_COM8 40
#define CON_COM9 41
#define CON_COM10 42
#define CON_COM11 43
#define CON_COM12 44
#define CON_COM13 45
#define CON_COM14 46
#define CON_COM15 47
#define CON_SERIALDEVICE_FNAME 63

#define PINMODE_NC0		0 //not connected (due to Simulink compatibility)
#define PINMODE_NC		1 //not connected
#define PINMODE_DO		2 //digital output
#define PINMODE_DI		3 //digital input
#define PINMODE_DIP		4 //digital input with internal 20k pull-up resistors 
#define PINMODE_PWM		5 //PWM output (analog output)
#define PINMODE_AI		6 //analog input
#define PINMODE_OW		7 //1-wire bus gateway
#define PINMODE_CNT		8 //counter input (counting falling edges by default)
#define PINMODE_ENC		9 //encoder input, signal A (evaluated on both edges of the A signal)
#define PINMODE_ENCB	10 //encoder input, signal B (digital input but no data is polled) 

#define MAX_PIN_COUNT 96 //size of internal arrays, must be greater than number of pins on the most populated board
#define MAX_PINMASK_BYTESIZE 12 // size of pinmask in bytes (1 byte..8 pins), i.e. ceil(MAX_PIN_COUNT/8.0)

long hCom = -1;
long sent = 0;
long commandData[COMMAND_LENGTH_MAX];
long responseData[RESPONSE_BUFFER_LENGTH];
long pinModes[MAX_PIN_COUNT], pinModes_par[MAX_PIN_COUNT];
long responseCnt = 0;
long sentCnt = 0;
double portOpenTime, initAttemptTime, lastSuccess, execTime;
long connected=0;
long initialized=0;
long pinsSync=0;
long REXduinoError = 0;
long PIN_COUNT, PINMASK_BYTESIZE;
long digitalIn[MAX_PIN_COUNT];
long analogIn[MAX_PIN_COUNT];
long oneWire[MAX_PIN_COUNT];
long digitalOut[MAX_PIN_COUNT];
long analogOut[MAX_PIN_COUNT];
long longArray[10]; //array of various values of type long (counter, user data, etc.)
long debugLast = 0; //state of the debug input in the previous tick
long TRACE_INCOMING, TRACE_OUTGOING; //tracing of serial communication for debugging purposes
 
long input(0) input0;
long input(1) input1;
long input(2) input2;
long input(3) input3;
long input(4) input4;
long input(5) input5;
long input(6) input6;
long input(7) input7;
long input(9) comPortNo;
long input(10) SimulinkDetector;
long input(13) debug;
long input(14) CNTcmd;
long input(15) userSend;

long output(0) output0;
long output(1) output1;
long output(2) output2;
long output(3) output3;
long output(4) output4;
long output(5) output5;
long output(6) output6;
long output(7) output7;
long output(8) output8;
long output(9) output9;
long output(10) output10;
long output(11) output11;
long output(12) output12;
long output(13) output13;
long output(14) output14;
long output(15) userRecv;

string parameter(0) comPort;
long parameter(1) param1;
long parameter(2) param2;
long parameter(3) param3;
long parameter(4) param4;
long parameter(5) param5;
long parameter(6) param6;
long parameter(7) param7;
long parameter(8) param8;
long parameter(9) param9;
long parameter(10) param10;
long parameter(11) param11;
long parameter(12) param12;
long parameter(13) param13;
long parameter(14) param14;
long parameter(15) maskType;

void traceSentData(long count)
{ //print to logfile the data which are sent to Arduino 
	long i;
if(TRACE_OUTGOING)
{
	for(i=0;i<count;i++)
	{
		Trace(11,commandData[i]);
	}
}
else
{
	i=0;
}
return;
}

void initCommunication(void)
{ //initialize communication with Arduino
	commandData[0]='C';
	commandData[1]=0;
	commandData[2]=';';
	sent = Send(hCom,commandData,3);
	sentCnt = sentCnt+3;
	traceSentData(3);
return;
}

void setDigitalOutput(long pin, long data[])
{ //set digital output
	commandData[0]='O';
	commandData[1]=pin;
	commandData[2]=data[pin];
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void setDigitalOutputMulti(long mask[], long data[], long maskByteSize)
{ //multi-set digital output
	long i;
	commandData[0]=1;
	for (i=0;i<maskByteSize;i++)
	{
		commandData[i+1]=mask[i];
		commandData[i+maskByteSize+1]=data[i];
	}
	commandData[2*maskByteSize+1]=';';
	sent = Send(hCom,commandData,2*maskByteSize+2);
	traceSentData(2*maskByteSize+2);
return;
}

void readDigitalInput(long pin)
{ //read digital input
	commandData[0]='I';
	commandData[1]=pin;
	commandData[2]=';';
	sent = Send(hCom,commandData,3);
	sentCnt = sentCnt+3;
	traceSentData(3);
return;
}

void readDigitalInputMulti(long mask[], long maskByteSize)
{ //multi-read digital input
	long i;
	commandData[0]=2;
	for (i=0;i<maskByteSize;i++)
	{
		commandData[i+1]=mask[i];
	}
	commandData[maskByteSize+1]=';';
	sent = Send(hCom,commandData,maskByteSize+2);
	traceSentData(maskByteSize+2);
return;
}

void setAnalogOutput(long pin, long data[])
{ //set PWM (analog) output
	commandData[0]='P';
	commandData[1]=pin;
	commandData[2]=data[pin];
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void readAnalogInput(long pin)
{ //read analog input
	commandData[0]='A';
	commandData[1]=pin;
	commandData[2]=';';
	sent = Send(hCom,commandData,3);
	sentCnt = sentCnt+3;
	traceSentData(3);
return;
}

void readOnewireTemp(long pin)
{ //read temperature from next 1-Wire device
	commandData[0]='T';
	commandData[1]=pin;
	commandData[2]=';';
	sent = Send(hCom,commandData,3);
	sentCnt = sentCnt+3;
	traceSentData(3);
return;
}

void readOnewireTempMulti(long mask[], long maskByteSize)
{ //multi-read temperature from next 1-Wire device
	long i;
	commandData[0]=3;
	for (i=0;i<maskByteSize;i++)
	{
		commandData[i+1]=mask[i];
	}
	commandData[maskByteSize+1]=';';
	sent = Send(hCom,commandData,maskByteSize+2);
	traceSentData(maskByteSize+2);
return;
}

void readCounter(long pin, long resetflag, long enableflag)
{ //read counter value
	commandData[0]='N';
	commandData[1]=pin;
	commandData[2]=resetflag*2+enableflag;
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void enableCounter(long pin)
{ //enable counter
	commandData[0]='N';
	commandData[1]=pin;
	commandData[2]='E';
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void disableCounter(long pin)
{ //read counter value
	commandData[0]='N';
	commandData[1]=pin;
	commandData[2]='D';
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void resetCounter(long pin)
{ //reset counter value
	commandData[0]='N';
	commandData[1]=pin;
	commandData[2]='R';
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
return;
}

void setDigiPotentiometer(long addrI2C, long resistorValues)
{ //set DS1844 digital potentiometer
	commandData[0]='R';
	commandData[1]=addrI2C;
	commandData[2]=resistorValues & 63;
	commandData[3]=resistorValues>>8 & 63;
	commandData[4]=resistorValues>>16 & 63;
	commandData[5]=resistorValues>>24 & 63;
	commandData[6]=';';
	sent = Send(hCom,commandData,7);
	sentCnt = sentCnt+7;
	traceSentData(7);
return;
}

void readBarometer(void)
{ //read barometer
	commandData[0]='B';
	commandData[1]=';';
	sent = Send(hCom,commandData,2);
	sentCnt = sentCnt+2;
	traceSentData(2);
return;
}

void userCommand(long micros)
{ //user command
	commandData[0]='U';
	if (micros!=0) {
		commandData[1]=1;
	}
	else{
		commandData[1]=0;
	}
	commandData[2]=';';
	sent = Send(hCom,commandData,3);
	sentCnt = sentCnt+3;
	traceSentData(3);
return;
}

int initPin(long pin)
{
	commandData[0]='M';
	commandData[1]=pin;
	switch (pinModes_par[pin]){
	case PINMODE_NC:
		commandData[2]='N';
		break;
	case PINMODE_DI:
		commandData[2]='I';
		break;
	case PINMODE_DIP:
		commandData[2]='J';
		break;
	case PINMODE_DO:
		if (digitalOut[pin]==0){
			commandData[2]='O';
		}
		else      {  
			commandData[2]='Q';
		}
		break;
	case PINMODE_AI:
		commandData[2]='A';
		break;
	case PINMODE_PWM:
		commandData[2]='P';
		break;
	case PINMODE_OW:
		commandData[2]='W';
		oneWire[pin]=0;
		break;
	case PINMODE_CNT:
		commandData[2]='C';
		break;
	case PINMODE_ENC:
		commandData[2]='E';
		break;
	case PINMODE_ENCB:
		commandData[2]='B';
		break;
	}
	commandData[3]=';';
	sent = Send(hCom,commandData,4);
	sentCnt = sentCnt+4;
	traceSentData(4);
	return 0;
}

int parchange(void)
{
	long i;
	for (i=0;i<MAX_PIN_COUNT;i++)
	{ //set all pins to "not connected" mode by default
		pinModes_par[i] = PINMODE_NC;
	}
	switch (maskType)
	{
	case MASKTYPE_ARD_UNO:
		PIN_COUNT = 20; //digital+analog pins
		PINMASK_BYTESIZE = (long)ceil(PIN_COUNT/8.0);
		pinModes_par[2] = param1;
		pinModes_par[3] = param2;
		pinModes_par[4] = param3;
		pinModes_par[5] = param4;
		pinModes_par[6] = param5;
		pinModes_par[7] = PINMODE_DIP;
		pinModes_par[8] = PINMODE_DIP;
		pinModes_par[9] = param6;
		pinModes_par[10] = param7;
		pinModes_par[11] = param8;
		pinModes_par[12] = PINMODE_DO;
		pinModes_par[13] = PINMODE_DO;
		pinModes_par[14] = param9;
		pinModes_par[15] = param10;
		pinModes_par[16] = param11;
		pinModes_par[17] = param12;
		pinModes_par[18] = param13;
		pinModes_par[19] = param14;
		break;
	case MASKTYPE_ARD_UNOhex:
		PIN_COUNT=20; //digital+analog pins
		PINMASK_BYTESIZE = (long)ceil(PIN_COUNT/8.0);
		for (i=0;i<8;i++)
		{ //
			pinModes_par[i]    = (param1 & (15<<(4*i))) >> 4*i; //pins 0 and 1 are overwritten below
			pinModes_par[i+8]  = (param2 & (15<<(4*i))) >> 4*i; //analog pins (14 and up) are overwritten below
		}
		pinModes_par[0]=PINMODE_NC;
		pinModes_par[1]=PINMODE_NC;
		for (i=0;i<8;i++)
		{ //analog pins A0 to A5 must be in separate for cycle (pin A0 = pin 14, which is not a multiple of 8)
			pinModes_par[i+14] = (param3 & (15<<(4*i))) >> 4*i;
		}
		break;
	case MASKTYPE_ARD_MEGA2560:
	case MASKTYPE_SEEED_MEGAV122:
		PIN_COUNT = 70; //digital+analog pins //the row of pins PE2 to to PH2 is not supported
		PINMASK_BYTESIZE = (long)ceil(PIN_COUNT/8.0);
		pinModes_par[2] = param1;
		pinModes_par[3] = param2;
		pinModes_par[4] = param3;
		pinModes_par[5] = param4;
		pinModes_par[6] = param5;
		pinModes_par[7] = PINMODE_DIP;
		pinModes_par[8] = PINMODE_DIP;
		pinModes_par[9] = param6;
		pinModes_par[10] = param7;
		pinModes_par[11] = param8;
		pinModes_par[12] = PINMODE_DO;
		pinModes_par[13] = PINMODE_DO;
		pinModes_par[54] = param9;
		pinModes_par[55] = param10;
		pinModes_par[56] = param11;
		pinModes_par[57] = param12;
		pinModes_par[58] = param13;
		pinModes_par[59] = param14;
		pinModes_par[60] = PINMODE_AI;
		pinModes_par[61] = PINMODE_AI;		
		break;
	case MASKTYPE_ARD_MEGA2560hex:
	case MASKTYPE_SEEED_MEGAV122hex:
		PIN_COUNT = 70; //digital+analog pins
		PINMASK_BYTESIZE = (long)ceil(PIN_COUNT/8.0);
		for (i=0;i<8;i++)
		{ //
			pinModes_par[i]    = (param1 & (15<<(4*i))) >> 4*i; //pins 0 and 1 are overwritten below
			pinModes_par[i+8]  = (param2 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+16] = (param3 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+24] = (param4 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+32] = (param5 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+40] = (param6 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+48] = (param7 & (15<<(4*i))) >> 4*i; //analog pins (54 and up) are overwritten below
		}
		pinModes_par[0]=PINMODE_NC;
		pinModes_par[1]=PINMODE_NC;
		for (i=0;i<8;i++)
		{ //analog pins A0 to A15 must be in separate for cycle (pin A0 = pin 54, which is not a multiple of 8)
			pinModes_par[i+54] = (param8 & (15<<(4*i))) >> 4*i;
			pinModes_par[i+62] = (param9 & (15<<(4*i))) >> 4*i;
		}
		break;
	} //end switch(maskType)
	for (i=0;i<MAX_PIN_COUNT;i++)
	{ 
		//convert PINMODE_NC0 to PINMODE_NC, due to Simulink compatibility
		if (pinModes_par[i] == PINMODE_NC0)
		{
			pinModes_par[i] = PINMODE_NC;
			pinModes[i] = PINMODE_NC;    
		}
	}
	return 0;
}

int initPort(void)
{
	long handle, i;
	long dropData[1];
  long buffer[50];
	//reset pinmodes to force reinit
	for (i=0;i<MAX_PIN_COUNT;i++)        
	{
		pinModes[i] = PINMODE_NC;
	}
		if (SimulinkDetector)
		{
      switch (comPortNo)
	{ //open COM port, returns handle>=0 if OK, otherwise REX error is returned (<0)
	case 1:
		handle = Open(CON_COM1,"",COM_BAUDRATE,0);
    Trace(1,1); 
		break;
	case 2:
		handle = Open(CON_COM2,"",COM_BAUDRATE,0); 
		Trace(1,2);
    break;
	case 3:
		handle = Open(CON_COM3,"",COM_BAUDRATE,0); 
		break;
	case 4:
		handle = Open(CON_COM4,"",COM_BAUDRATE,0); 
		break;
	case 5:
		handle = Open(CON_COM5,"",COM_BAUDRATE,0); 
		break;
	case 6:
		handle = Open(CON_COM6,"",COM_BAUDRATE,0); 
		break;
	case 7:
		handle = Open(CON_COM7,"",COM_BAUDRATE,0); 
		break;
	case 8:
		handle = Open(CON_COM8,"",COM_BAUDRATE,0); 
		break;
	case 9:
		handle = Open(CON_COM9,"",COM_BAUDRATE,0); 
		break;
	case 10:
		handle = Open(CON_COM10,"",COM_BAUDRATE,0); 
		Trace(1,10);
    break;
	case 11:
		handle = Open(CON_COM11,"",COM_BAUDRATE,0);
    Trace(1,11); 
		break;
	case 12:
		handle = Open(CON_COM12,"",COM_BAUDRATE,0);
    Trace(1,12); 
		break;
	case 13:
		handle = Open(CON_COM13,"",COM_BAUDRATE,0);
    Trace(1,13); 
		break;
	case 14:
		handle = Open(CON_COM14,"",COM_BAUDRATE,0);
    Trace(1,14); 
		break;
	case 15:
		handle = Open(CON_COM15,"",COM_BAUDRATE,0);
    Trace(1,15); 
		break;
	default:
    handle = -1;
    Trace(1,-1);	
	} //end switch com_port

    }
    else
    {
    handle = OpenCom(comPort,COM_BAUDRATE,0);
    Trace(1,comPort);
    } 
	buffer[0] = 0;
  if(handle>=0)
	{
		GetOptions(handle, buffer);   //read serial port parameters; for details see e.g. MSDN functions SetComState and SetCommTimeouts

/*
		for (i=0;i<31;i++)
		{ //print com port options to logfile
		Trace(332,buffer[i]);
		}

		buffer[0]=COM_BAUDRATE;    //BaudRate 
		buffer[1]=0;				//fParity
		buffer[2]=0;				//Parity
		buffer[3]=0;				//StopBits
		buffer[4]=8;        //ByteSize
		buffer[5]=0;				//fDtrControl
		buffer[6]=0;				//fRtsControl
		buffer[7]=0;				//fAbortOnError
		buffer[8]=1;        //fBinary
		buffer[12]=0;               //fInX
		buffer[13]=0;               //fOutX		
		buffer[14]=0;               //fOutxCtsFlow
		buffer[15]=0;               //fOutxDsrFlow
		SetOptions(handle,buffer);   //set serial port parameters
		GetOptions(handle, buffer);   //read serial port parameters; for details see e.g. MSDN functions SetComState and SetCommTimeouts
		
		for (i=0;i<31;i++)
		{ //print com port options to logfile
		Trace(333,buffer[i]);
		}
*/		
//purge serial buffer by reading data which were in the serial buffer before opening the port
		dropData[0] = 0;
    while ((i=Recv(handle,dropData,1))>0)
		{
      Trace(2222,dropData[0]);
    }
	}
	else REXduinoError = ERROR_OPENING_PORT;
	parchange(); //read pinmodes from parameters
	return handle;
}

int closePort(long handle)
{
if(handle>=0) Close(handle);
	Trace(2,0);
				handle = -1;
			initialized = 0;
			portOpenTime = CurrentTime();
			responseCnt = 0;         
	return handle;
}

int init(void)
{
	// initialization commands executed once at startup of REXLANG
	hCom = initPort(); //calls also parchange()
	portOpenTime = CurrentTime();
	lastSuccess = CurrentTime();
	initAttemptTime=CurrentTime();
	execTime=CurrentTime();
	return 0;
}

int main(void)
{
	long lastIn[1];
	long i, k, l;
	long Padc, Tadc;
	double a0, b1, b2, c12;
	long outputData1, outputData2, outputData3;
	long countDigitalIn=0;
	long countDigitalOut=0;
	long countOnewireBranch=0;
	long digitalInMask[MAX_PINMASK_BYTESIZE]; 
	long digitalOutMask[MAX_PINMASK_BYTESIZE];
	long digitalOutMulti[MAX_PINMASK_BYTESIZE];
	long readTempMask[MAX_PINMASK_BYTESIZE];
  long debugChange;
	REXduinoError = 0;
	sentCnt = 0; //counter of sent bytes
  
  TRACE_INCOMING = (debug>>2) & 1;
  TRACE_OUTGOING = (debug>>3) & 1;
  
	//first we demux the data entering REXLANG (data for Arduino outputs)
	for (i=0;i<32;i++)
	{
		digitalOut[i] = ((input0&(1<<i)) >> i) & 1;
		digitalOut[i+32] = ((input1&(1<<i)) >> i) & 1;
		digitalOut[i+64] = ((input2&(1<<i)) >> i) & 1;
	}
	for (i=2;i<6;i++) //pins 0 and 1 are occupied by serial line
	{
		analogOut[i] = (( input3 & (255 << ((i-2)*8)) ) >> (i-2)*8) & 255;
		analogOut[i+4] = (( input4 & (255 << ((i-2)*8)) ) >> (i-2)*8) & 255;
		analogOut[i+8] = (( input5 & (255 << ((i-2)*8)) ) >> (i-2)*8) & 255;
		analogOut[i+12] = (( input6 & (255 << ((i-2)*8)) ) >> (i-2)*8) & 255;
		analogOut[i+42] = (( input7 & (255 << ((i-2)*8)) ) >> (i-2)*8) & 255;
	}
	//arrays of data for Arduino outputs are ready

	if (hCom<0)
	{ //serial port not open
		if (ElapsedTime(CurrentTime(),portOpenTime)>COM_REOPEN_INTERVAL*0.001)
		{
			hCom = initPort();
			portOpenTime = CurrentTime();
		}
		lastSuccess = CurrentTime();
		initAttemptTime=CurrentTime();
	}
	else 
	{ //serial port open
		if (!initialized) 
		{ //communication between REX and Arduino not established yet
			if (ElapsedTime(CurrentTime(),initAttemptTime)>COMM_INIT_INTERVAL*0.001)
			{
        initCommunication();
				initAttemptTime=CurrentTime();
			}
			lastSuccess = CurrentTime();
		}
    lastIn[0] = 0;
		while ((Recv(hCom,lastIn,1))>0)
		{ // first we process incoming serial data
			responseCnt++;
			responseData[responseCnt-1] = lastIn[0];
      if (TRACE_INCOMING)
      {
			   Trace(88,lastIn[0]); //print all received data to logfile
      }
			if (lastIn[0]==';')
			{
				/*
				for (i=0;i<responseCnt;i++) //print received data to logfile
				{
				Trace(33,responseData[i]);
				}
				Trace(222,99.9);
				*/
				switch (responseData[0])
				{
				case 'C': //connection confirmation
					if ((responseCnt==3) && (responseData[1]==48)) //48 is the ASCII value of '0'
					{      
						initialized = 1;
						REXduinoError = 0;
						responseCnt = 0;
						lastSuccess=CurrentTime();
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'E': //error or warning message
					if (responseCnt==4)
					{      
						if (responseData[1]<48) 
						{ //warning only
							Trace(8,responseData[1]); //print warning to system log
							Trace(8,responseData[2]); //print warning to system log
						}
						else 
						{ // error message
							if ( (responseData[1]==52) && (responseData[2]==50) ){
								//1-Wire bus busy, this error message is ignored
							}
							else
							{
								REXduinoError = (responseData[1]<<8)|responseData[2];
								Trace(9,responseData[1]); //print error to system log
								Trace(9,responseData[2]); //print error to system log
							}
						}
						responseCnt = 0;
						//lastSuccess=CurrentTime();
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'M':  //pin mode confirmation
					if ((responseCnt==4))
					{      
						if (responseData[1]>=2 && responseData[1]<=PIN_COUNT)
            {
            switch(responseData[2])
						{
						case 'N':
							pinModes[responseData[1]] = PINMODE_NC; 
							break;
						case 'I':
							pinModes[responseData[1]] = PINMODE_DI; 
							break;
						case 'J':
							pinModes[responseData[1]] = PINMODE_DIP; 
							break;
						case 'O':
						case 'Q':
							pinModes[responseData[1]] = PINMODE_DO; 
							break;
						case 'P':
							pinModes[responseData[1]] = PINMODE_PWM; 
							break;
						case 'A':
							pinModes[responseData[1]] = PINMODE_AI; 
							break;
						case 'W':
							pinModes[responseData[1]] = PINMODE_OW; 
							break;
						case 'C':
							pinModes[responseData[1]] = PINMODE_CNT; 
							break;
						case 'E':
							pinModes[responseData[1]] = PINMODE_ENC; 
							break;
						case 'B':
							pinModes[responseData[1]] = PINMODE_ENCB; 
							break;
						}
						lastSuccess=CurrentTime();
            }
            else
            {
              //received confirmation from non-existing pin
              REXduinoError = ERROR_RESPONSE_INVALID;
      					for (i=0;i<responseCnt;i++)
			       		{
						      Trace(7779,responseData[i]); //print error to system log          
					      }
            }
						responseCnt = 0;
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'I':  //digital input
					if (responseCnt==4)
					{
						digitalIn[responseData[1]] = responseData[2]; 
						lastSuccess=CurrentTime();
						responseCnt = 0;
					}		
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'A':  //analog input
					if (responseCnt==5)
					{
						analogIn[responseData[1]] = responseData[2]<<8 | responseData[3]; 
						lastSuccess=CurrentTime();
						responseCnt = 0;
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'O':  //digital output
				case 'P':  //analog output (PWM)
					if (responseCnt==4)
					{
						lastSuccess=CurrentTime(); 
						responseCnt = 0;
					}		
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'T':  //1-Wire temperature
					if (responseCnt==4)
					{ //
						switch (responseData[2])
						{
						case STATUS_ONEWIRE_NOMOREDEVICES:
							initPin(responseData[1]);
							responseCnt = 0;
							lastSuccess=CurrentTime();
							break;
						case STATUS_ONEWIRE_TEMPCONV:
							responseCnt = 0;
							lastSuccess=CurrentTime();
							break;
						default:
							//unknown message
						}
					}
					else if (responseCnt==7)
					{
						oneWire[responseData[1]] = (responseData[3] & 15)<<12 | (responseData[5] & 15)<<8 | responseData[4];    
						responseCnt = 0;
						lastSuccess=CurrentTime();
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'N':  //counter or encoder value
					if (responseCnt==8)
					{
						switch (responseData[1])
						{
						case 2:
							longArray[2] = responseData[3]<<0 | responseData[4]<<8 | responseData[5]<<16 | responseData[6]<<24;    
							responseCnt = 0;
							lastSuccess=CurrentTime();
							break;
						case 3:
							longArray[3] = responseData[3]<<0 | responseData[4]<<8 | responseData[5]<<16 | responseData[6]<<24;
							responseCnt = 0;
							lastSuccess=CurrentTime();
							break;
						default:
							//invalid counter ID, counters only on pins 2 and 3
						}
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'B':  //barometer data
					if (responseCnt==14)
					{
						Padc = (((responseData[1]<<8)|(responseData[2])))>>6;
						Tadc = (((responseData[3]<<8)|(responseData[4])))>>6;
						a0 = (((responseData[5]<<24)|(responseData[6]<<16))>>16)/8.0;
						b1 = ((((responseData[7]<<24)|(responseData[8]<<16))>>16)>>6)/128.0;
						b2 = (((responseData[9]<<24)|(responseData[10]<<16))>>16)/16384.0;
						c12 = ((((responseData[11]<<24)|(responseData[12]<<16))>>16)>>2)/4194304.0;
						longArray[0] = (long)floor(a0 + (b1 + c12 * Tadc) * Padc + b2 * Tadc + 0.5); // the +0.5 is a trick used for rounding
						responseCnt = 0;
						lastSuccess = CurrentTime();
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'R':  //digital potentiometer
					if (responseCnt==3)
					{
						lastSuccess=CurrentTime(); 
						responseCnt = 0;
					}		
					else
					{
						//incomplete or invalid response
					}
					break;
				case 'U':  //response to user-function call
					if (responseCnt==6)
					{
						longArray[0] = responseData[1] + (responseData[2]<<8) + (responseData[3]<<16) + (responseData[4]<<24);
						lastSuccess=CurrentTime(); 
						responseCnt = 0;
					}
					else
					{
						//incomplete or invalid response
					}
					break;
				default: //unknown response
					REXduinoError = ERROR_RESPONSE_INVALID;
					for (i=0;i<responseCnt;i++)
					{
						Trace(7777,responseData[i]); //print error to system log          
					}
					responseCnt = 0;
				} //end switch responseData[0]
			} // if lastIn
			else if ( responseCnt >= (RESPONSE_BUFFER_LENGTH-3) ) //invalid data received, too much bytes and no closing ';'
			{ 
				REXduinoError = ERROR_RESPONSE_INVALID;
				for (i=0;i<responseCnt;i++)
				{
					Trace(7778,responseData[i]); //print error to system log          
				}
				responseCnt = 0; 
			}
		} //while incoming data

		// here j==0 might mean that Arduino is disconnected, but I'm not sure if it works on all platforms
		//"Arduino disconnected" or no "valid response received within timeout period" or "opening of port succeeded but the communication initialization did not proceed"
		if ( (ElapsedTime(CurrentTime(),lastSuccess)>(TIMEOUT_FAC*GetPeriod())) || ((ElapsedTime(CurrentTime(),portOpenTime)>COM_REOPEN_INTERVAL*0.001) && (!initialized)) )
		{ 
			hCom = closePort(hCom);
		}
		if (initialized)
		{ //if connection is established, we can initialize pins and/or send commands 
			for (i=0;i<MAX_PINMASK_BYTESIZE;i++)
			{
				digitalInMask[i]=0;
				digitalOutMask[i]=0;
				readTempMask[i]=0;
				digitalOutMulti[i]=0;
			}
			pinsSync = 1;
			for (i=2;i<PIN_COUNT;i++) //first we init and count pins
			{
				if (pinModes[i]!=pinModes_par[i]) 
				{
					initPin(i);
					pinsSync = 0;
				} 
				else 
				{
					switch (pinModes[i])
					{
					case PINMODE_DO:
						k = (long)ceil(fmod(i,8));
						l = (long)floor(i/8.0);
						digitalOutMask[l] = digitalOutMask[l] | 1<<k; //only bit mask is created here
						digitalOutMulti[l] = digitalOutMulti[l] | ((1&digitalOut[i])<<k); 
						countDigitalOut++;
						break;
					case PINMODE_DI:
					case PINMODE_DIP:
						k = (long)ceil(fmod(i,8));
						l = (long)floor(i/8.0);
						digitalInMask[l] = digitalInMask[l] | 1<<k; //only bit mask is created here
						countDigitalIn++;
						break;
					case PINMODE_OW:
						k = (long)ceil(fmod(i,8));
						l = (long)floor(i/8.0);
						readTempMask[l] = readTempMask[l] | 1<<k; //only bit mask is created here, temperature conversion is initiated after all other commands, otherwise losses of serial data might occur in large applications
						countOnewireBranch++;
						break;
					} //end switch pinModes[i]
				}  
			}
			if (pinsSync)
			{
				// put user-defined commands here *****************************************
				userCommand(userSend);
				//readBarometer();
				//setDigiPotentiometer(0,userSend); //I2C address is 0, potentiometer values are given by the userSend input
				// end of user-defined commands *******************************************
				for (i=2;i<PIN_COUNT;i++) //and now we send commands
				{
					switch (pinModes[i])
					{
					case PINMODE_DO:
						if ( (countDigitalOut*4) < (PINMASK_BYTESIZE*2+2) )
						{ //only a few digital outputs
							setDigitalOutput(i, digitalOut);	
						}
						break;
					case PINMODE_DI:
					case PINMODE_DIP:
						if ( (countDigitalIn*3) < (PINMASK_BYTESIZE+2) )
						{ //only a few digital inputs
							readDigitalInput(i);						
						}
						break;
					case PINMODE_PWM:
						setAnalogOutput(i, analogOut);
						break;
					case PINMODE_AI:
						readAnalogInput(i);
						break;
					case PINMODE_OW:
						if ( (countOnewireBranch*3) < (PINMASK_BYTESIZE+2) )
						{ //only a few 1-Wire branches
							readOnewireTemp(i);						
						}
						break;
					case PINMODE_ENC:
					case PINMODE_CNT:
						readCounter(i, ( (CNTcmd & (long)floor((pow(2,2*(i-2)))+0.5))>0 ? 1 : 0 ), ( (CNTcmd & (long)floor((pow(2,2*(i-2)+1))+0.5))>0 ? 1 : 0 ) ); // pin number, reset flag, enable flag
						break;
					} //end switch pinModes[i]
				}
				if ( (countDigitalOut*4) >= (PINMASK_BYTESIZE*2+2) )
				{ //multi-command is less data-intensive
					setDigitalOutputMulti(digitalOutMask, digitalOutMulti, PINMASK_BYTESIZE);	
				}
				if ( (countDigitalIn*3) >= (PINMASK_BYTESIZE+2) )
				{
					readDigitalInputMulti(digitalInMask, PINMASK_BYTESIZE);	
				}
				if ( (countOnewireBranch*3) >= (PINMASK_BYTESIZE+2) )
				{
					readOnewireTempMulti(readTempMask, PINMASK_BYTESIZE);	
				}
			}
			else //pinmodes on master and slave do not match
			{
				//no commands are sent, until all changes in pinmodes are confirmed by REXduino slave      
			}
		}        
	} //if-else hCom<0

	if (hCom>=0)
	{
		connected=1;	
	}
	else 
	{
		connected=0;
	}

	//now we combine the data which leave the REXLANG block
	outputData1 = 0;
	outputData2 = 0;
	outputData3 = 0;
	for (i=0;i<32;i++)
	{
		outputData1 = outputData1|digitalIn[i]<<i;
		outputData2 = outputData2|digitalIn[i+32]<<i;
		outputData3 = outputData3|digitalIn[i+64]<<i;
	}
	output0 = outputData1;
	output1 = outputData2;
	output2 = outputData3;
  output8 = REXduinoError | ((long)((connected && !initialized)||(connected && pinsSync)))<<16 | initialized<<17 | analogIn[69]<<22;  
	switch (maskType)
	{
	case MASKTYPE_ARD_UNO:
		//1-Wire branches on pins 2-6, 9-11, A0-A3
		output3 = analogIn[14] | analogIn[15]<<10 | analogIn[16]<<20; 
		output4 = analogIn[17] | analogIn[18]<<10 | analogIn[19]<<20;
		output9 = oneWire[2] | oneWire[3]<<16;
		output10 = oneWire[4] | oneWire[5]<<16;
		output11 = oneWire[6] | oneWire[9]<<16;
		output12 = oneWire[10] | oneWire[11]<<16;
		output13 = oneWire[14] | oneWire[15]<<16;
		output14 = oneWire[16] | oneWire[17]<<16;    
		output6 = longArray[2]; //counter on pin 2 
		output7 = longArray[3]; //counter on pin 3 
		break;
	case MASKTYPE_ARD_UNOhex:
		//1-Wire branches on pins 2-12, A0-A5
    output1 = analogIn[14] | analogIn[15]<<10 | analogIn[16]<<20; 
		output2 = analogIn[17] | analogIn[18]<<10 | analogIn[19]<<20;
		output3 = oneWire[2] | oneWire[3]<<16;
		output4 = oneWire[4] | oneWire[5]<<16;
		output5 = oneWire[6] | oneWire[7]<<16;
		output9 = oneWire[8] | oneWire[9]<<16;
		output10 = oneWire[10] | oneWire[11]<<16;
    output11 = oneWire[12]; 
    output12 = oneWire[14] | oneWire[15]<<16;
		output13 = oneWire[16] | oneWire[17]<<16;
		output14 = oneWire[18] | oneWire[19]<<16;    
		output6 = longArray[2]; //counter on pin 2 
		output7 = longArray[3]; //counter on pin 3 
		break;
	case MASKTYPE_ARD_MEGA2560:
	case MASKTYPE_SEEED_MEGAV122:
		//1-Wire branches on pins 2-6, 9-11, A0-A3
		output3 = analogIn[54] | analogIn[55]<<10 | analogIn[56]<<20; 
		output4 = analogIn[57] | analogIn[58]<<10 | analogIn[59]<<20;
		output5 = analogIn[60] | analogIn[61]<<10;		
		output9 = oneWire[2] | oneWire[3]<<16;
		output10 = oneWire[4] | oneWire[5]<<16;
		output11 = oneWire[6] | oneWire[9]<<16;
		output12 = oneWire[10] | oneWire[11]<<16;
		output13 = oneWire[54] | oneWire[55]<<16;
		output14 = oneWire[56] | oneWire[57]<<16;
		output6 = longArray[2]; //counter on pin 2  
		output7 = longArray[3]; //counter on pin 3  
		break;		
	case MASKTYPE_ARD_MEGA2560hex:
	case MASKTYPE_SEEED_MEGAV122hex:
		//analog inputs
		output3 = analogIn[54] | analogIn[55]<<10 | analogIn[56]<<20; 
		output4 = analogIn[57] | analogIn[58]<<10 | analogIn[59]<<20;
		output5 = analogIn[60] | analogIn[61]<<10 | analogIn[62]<<20;
		output6 = analogIn[63] | analogIn[64]<<10 | analogIn[65]<<20;
		output7 = analogIn[66] | analogIn[67]<<10 | analogIn[68]<<20;
		// by default, 1-wire bus is on pins 22 to 33
		output9 = oneWire[22] | oneWire[23]<<16;
		output10 = oneWire[24] | oneWire[25]<<16;
		output11 = oneWire[26] | oneWire[27]<<16;
		output12 = oneWire[28] | oneWire[29]<<16;
		output13 = oneWire[30] | oneWire[31]<<16;
		output14 = oneWire[32] | oneWire[33]<<16;
		/*
		// use this code if you want to use pins 2-13 for 1-wire bus 
		output9 = oneWire[2] | oneWire[3]<<16;
		output10 = oneWire[4] | oneWire[5]<<16;
		output11 = oneWire[6] | oneWire[7]<<16;
		output12 = oneWire[8] | oneWire[9]<<16;
		output13 = oneWire[10] | oneWire[11]<<16;
		output14 = oneWire[12] | oneWire[13]<<16;
		*/
		if ((pinModes[2]==PINMODE_ENC) | (pinModes[2]==PINMODE_CNT))
		{  
			output9 = longArray[2];  //counter on pin 2 
		}
		if ((pinModes[3]==PINMODE_ENC) | (pinModes[3]==PINMODE_CNT))
		{  
			output10 = longArray[3]; //counter on pin 3 
		}
		break;
	}
	userRecv = longArray[0]; //output available for user data

  debugChange = debug & ~debugLast; //detection of rising edge (bitwise)
  if (debugChange & 1) //force reconnect by closing the serial port
  {
 			hCom = closePort(hCom);
  }
  if ((debugChange>>1) & 1) //print pinmodes to system log 
  {
    Trace(555,5555.5);
    for (i=2;i<PIN_COUNT;i++)
    {
      Trace(550,i);
      Trace(551,pinModes_par[i]);
      Trace(552,pinModes[i]);
    }   
    Trace(555,5555.5);
  }
  if ((debugChange>>4) & 1) //print REXduino master version to system log 
  {
    Trace(444,4444.4);
    Trace(440,MAJORVERSION);
    Trace(441,MINORVERSION);
    Trace(442,REVISION);
    Trace(442,COMMIT);
    Trace(444,4444.4);
  }

  debugLast = debug;

if(TRACE_OUTGOING||TRACE_INCOMING)
{
	Trace(9999,999999.9);
}
	return 0; //end main
}

int exit(void)
{
	hCom = closePort(hCom);
	return 0;
}