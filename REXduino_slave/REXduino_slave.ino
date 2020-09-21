/*********************************************************
 *                                                       *
    REXduino slave
    REXYGEN and Arduino communication over serial port
 *                                                       *
    Jaroslav SOBOTA, 2013-2020
    email: jsobota@ntis.zcu.cz
 *                                                       *
 *********************************************************/

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

#define MAJORVERSION 2
#define MINORVERSION 0 //even number = release, odd number = development
#define REVISION 3 //for hotfixes, even number = hotfix applied, odd number = development
#define COMMIT 0 //

//#define USE1WIRE //uncomment this line to use 1-Wire temperature sensors, OneWire library is required
//#define USEI2C //uncomment this line to use I2C bus
//#define USESERVO //uncomment this line to use RC servos

// uncomment the line below to use watchdog timer (8s by default, see the setup() section)
//     - works on Arduino UNO R2, R3
//     - after updating bootloader, WORKS on Arduino MEGA2560 (see Nick Gammon's instructions at http://www.gammon.com.au/bootloader )
//     - after updating bootloader, WORKS on MEGA328P (Arduino Nano, Seeeduino v2.21), https://github.com/Optiboot/optiboot
//     - other boards have not been tested, please let me know
//#define ENABLE_WDT

#ifdef USE1WIRE
#include <OneWire.h>
#endif

#ifdef USEI2C
#include <Wire.h>
#endif

#ifdef ENABLE_WDT
#include <avr/wdt.h>
#endif

#define COM_BAUDRATE 57600 //serial port baudrate
#define COMM_TIMEOUT 3000 //timeout for receiving a complete and valid command (milliseconds)
#define COMMAND_LENGTH_MAX 30 //maximum length of command 
#define ONEWIRE_TEMPCONV_DELAY 800 //time required for 1-Wire temperature conversion
#define BUTTON_DEBOUNCE_MILLIS 20 //debouncing interval for counting button presses

#define ERROR_COMMAND             49 //1
#define ERROR_COMMAND_TIMEOUT       49 //1
#define ERROR_COMMAND_UNKNOWN       50 //2
#define ERROR_COMMAND_INVALID       51 //3
#define ERROR_COMMAND_PIN_NUMBER    52 //4
#define ERROR_PINMODE             50 //2
#define ERROR_PINMODE_UNKNOWN       49 //1
#define ERROR_PINMODE_INVALID       50 //2
#define ERROR_SERIAL              51 //3
#define ERROR_SERIAL_BUFFER_FULL    49 //1
#define ERROR_ONEWIRE             52 //4
#define ERROR_ONEWIRE_NOMOREDEVICES 49 //1
#define ERROR_ONEWIRE_BUSYBUS       50 //2
#define ERROR_ONEWIRE_BADCRC        51 //3
#define ERROR_ONEWIRE_DEPLETED      52 //4
#define ERROR_ONEWIRE_NOTEMPDEVICE  53 //5
#define ERROR_I2C                 53 //5
#define ERROR_I2C_BAROMETER         49 //1
#define ERROR_I2C_BAROMETERDATA     50 //2
#define ERROR_I2C_RESISTOR          51 //3
#define ERROR_I2C_RESISTORDATA      52 //4

#define WARNING_SERIAL               1
#define WARNING_SERIAL_BUFFER_FULL      1

#define STATUS_ONEWIRE_OK             48 //0
#define STATUS_ONEWIRE_SENSORFOUND    49 //1
#define STATUS_ONEWIRE_NOMOREDEVICES  50 //2
#define STATUS_ONEWIRE_DEPLETED       51 //3
#define STATUS_ONEWIRE_TEMPCONV       52 //4
#define STATUS_ONEWIRE_INVALIDROM     53 //5

#define ACTION_POSTPONE_SWITCHOFF      1
#define ACTION_POSTPONE_SWITCHON       2
#define ACTION_POSTPONE_OWTEMP         3
#define ACTION_POSTPONE_OWTEMP_VERBOSE 4
#define ACTION_POSTPONE_REPORT_DEPLETED 5
#define ACTION_POSTPONE_REPORT_DEPLETED_VERBOSE 6

#define COMMAND_CNT_RESET  82 //R - counter reset
#define COMMAND_CNT_ENABLE 69 //E - counter enable
#define COMMAND_CNT_DISABLE 68 //D - counter disable
#define COMMAND_CNT_VALUE   86 //V - read counter value

byte master_active = 0;
unsigned long command_starttime = 0; //time when the reception of the last command started
byte command_data[COMMAND_LENGTH_MAX];
byte command_data_length = 0;
byte PINMASK_BYTESIZE;

byte onewire_deviceAddress[NUM_DIGITAL_PINS][8]; //addresses (8-byte ROM) of the active devices in individual 1-Wire branches
byte onewire_deviceOrder[NUM_DIGITAL_PINS]; //order numbers of the active devices in individual 1-Wire branches (starting at 1, ordered by 1-Wire ROM)
unsigned long onewire_busBlockingStart[NUM_DIGITAL_PINS]; //time when 1-Wire bus blocking started (parasitic devices need the bus to remain HIGH for some time)
int onewire_busBlockingLength[NUM_DIGITAL_PINS]; //length of the bus-HIGH interval required for proper function of the parasitic 1-Wire device
byte onewire_postponedAction[NUM_DIGITAL_PINS]; //actions to perform after 1-Wire bus is released

volatile long counter2 = 0; //counter on pin 2
volatile long counter3 = 0; //counter on pin 3

byte CNT2EN; //enable counter on pin 2
byte CNT3EN; //enable counter on pin 3
byte CNT2EN_old; //last state of ENABLE signal for edge detection
byte CNT3EN_old; //last state of ENABLE signal for edge detection
long CNT2hld; //value of counter to hold
long CNT3hld; //value of counter to hold

#ifdef USE1WIRE
OneWire  ds2(2), ds3(3), ds4(4), ds5(5), ds6(6), ds7(7), ds8(8), ds9(9);
OneWire  ds10(10), ds11(11), ds12(12), ds14(14), ds15(15), ds16(16), ds17(17), ds18(18), ds19(19);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
OneWire  ds20(20), ds21(21), ds22(22), ds23(23), ds24(24), ds25(25), ds26(26), ds27(27), ds28(28), ds29(29);
OneWire  ds30(30), ds31(31), ds32(32), ds33(33), ds34(34), ds35(35), ds36(36), ds37(37), ds38(38), ds39(39);
OneWire  ds40(40), ds41(41), ds42(42), ds43(43), ds44(44), ds45(45), ds46(46), ds47(47), ds48(48), ds49(49);
OneWire  ds50(50), ds51(51), ds52(52), ds53(53), ds54(54), ds55(55), ds56(56), ds57(57), ds58(58), ds59(59);
OneWire  ds60(60), ds61(61), ds62(62), ds63(63), ds64(64), ds65(65), ds66(66), ds67(67), ds68(68), ds69(69);
OneWire* onewire[70] = {
  0, 0, &ds2, &ds3, &ds4, &ds5, &ds6, &ds7, &ds8, &ds9,
  &ds10, &ds11, &ds12,  0  , &ds14, &ds15, &ds16, &ds17, &ds18, &ds19, // e.g. ow_ptr[5]->reset is equivalent to ds5.reset
  &ds20, &ds21, &ds22, &ds23, &ds24, &ds25, &ds26, &ds27, &ds28, &ds29,
  &ds30, &ds31, &ds32, &ds33, &ds34, &ds35, &ds36, &ds37, &ds38, &ds39,
  &ds40, &ds41, &ds42, &ds43, &ds44, &ds45, &ds46, &ds47, &ds48, &ds49,
  &ds50, &ds51, &ds52, &ds53, &ds54, &ds55, &ds56, &ds57, &ds58, &ds59,
  &ds60, &ds61, &ds62, &ds63, &ds64, &ds65, &ds66, &ds67, &ds68, &ds69
};
#else
OneWire* onewire[20] = {
  0, 0, &ds2, &ds3, &ds4, &ds5, &ds6, &ds7, &ds8, &ds9,
  &ds10, &ds11, &ds12, 0, &ds14, &ds15, &ds16, &ds17, &ds18, &ds19
}; // e.g. ow_ptr[5]->reset is equivalent to ds5.reset
#endif //board type

#endif //use 1-Wire

void setup()
{ //executed once at Arduino startup
  byte i;
  Serial.begin(COM_BAUDRATE); //initialize serial port communication
  /*
    Serial.println();
    Serial.println("Initializing...");
  */
  //Change the PWM frequency only if you are ABSOLUTELY SURE what you are doing !!!
  //Base frequency on pins 9 and 10 is 31250 Hz
  //The divisors available on pins 9 and 10 are: 1 (=31.25 KHz), 8 (~ 3.9 KHz), 64 (~ 488 Hz), 256 (~ 122 Hz), and 1024 (~ 30 Hz).
  //BEWARE, changing the PWM timing conflicts with the Servo library !!!
  //setPwmFrequency(9, 8); //for Arduino UNO, make sure to uncomment also the function itself in REXduino_functions.ino
  //For Arduino MEGA2560 - change PWM frequency of pins 9 and 10 on MEGA2560
  //Prescalers available on pins 9 and 10 are: 1 (=31.25 KHz), 2 (~ 3.9 KHz), 3 (~ 488 Hz), 4 (~ 122 Hz), and 5 (~ 30 Hz).
  //int myPrescaler = 2;
  //TCCR2B = (TCCR2B & ~7) | myPrescaler; //

#ifdef USEI2C
  Wire.begin(); // join i2c bus
#endif
#ifdef ENABLE_WDT
  wdt_enable(WDTO_8S);  // reset after 8 seconds, if no commands received
#endif
  for (i = 0; i < COMMAND_LENGTH_MAX; i++)
  { //initialize the buffer for command data
    command_data[i] = 0;
  }
  PINMASK_BYTESIZE = (long)ceil(NUM_DIGITAL_PINS / 8.0);
  detachInterrupt(0);
  detachInterrupt(1);
}

void loop()
{ //executed again and again
  byte verbose_command = 0;
  byte lastIn = 0;
  byte k;

  for (k = 0; k < NUM_DIGITAL_PINS; k++)
  { // let's check first if there are any pending operations to be performed
    if ((onewire_busBlockingLength[k] > 0) && ((millis() - onewire_busBlockingStart[k]) > onewire_busBlockingLength[k]))
    {
      switch (onewire_postponedAction[k])
      {
        case ACTION_POSTPONE_SWITCHOFF:
          digitalWrite(k, LOW);
          break;
        case ACTION_POSTPONE_SWITCHON:
          digitalWrite(k, HIGH);
          break;
#ifdef USE1WIRE
        case ACTION_POSTPONE_OWTEMP_VERBOSE:
          verbose_command = 1;
        case ACTION_POSTPONE_OWTEMP:
          readOnewireTemp(k, verbose_command);
          break;
        case ACTION_POSTPONE_REPORT_DEPLETED_VERBOSE:
          verbose_command = 1;
        case ACTION_POSTPONE_REPORT_DEPLETED:
          reportOnewireDepleted(k, verbose_command);
          break;
#endif
      }
      onewire_busBlockingLength[k] = 0;
    }
    verbose_command = 0;
  }

  while ( (Serial.available() > 0) && (lastIn != ';') && (command_data_length < COMMAND_LENGTH_MAX) )
  { //data is available and the command has not been completed yet
    if (command_data_length == 0)
    {
      command_starttime = millis();
    }
    lastIn = Serial.read();
    command_data[command_data_length] = lastIn;
    command_data_length++;
  }
  if (((millis() - command_starttime) > COMM_TIMEOUT) && (command_data_length > 0))
  { //correctly terminated command did not arrive within the given interval
    reportError(ERROR_COMMAND, ERROR_COMMAND_TIMEOUT);
    command_data_length = 0;
  }
  if (lastIn == ';')
  { //the incoming command might be complete
    byte reqLength = 0; //expected length of command
    switch (command_data[0]) {
      case 'c': // initialize communication
        verbose_command = 1;
      case 'C': // initialize communication
        reqLength = 3;
        if (command_data_length == reqLength)
        { //correctly terminated command of the expected length received
          commandC(command_data[1], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'C' command
      case 'v': // verbose version of the "version info" command
        verbose_command = 1;
      case 'V': // "version info" command
        reqLength = 2;
        if (command_data_length == reqLength)
        { //correctly terminated command of the expected length received
          commandV(verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'V' command
      case 'm': // verbose version of the "change pin mode" command
        verbose_command = 1;
      case 'M': // "change pin mode" command
        reqLength = 4;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandM(command_data[1], command_data[2], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'M' command
      case 'o': // verbose version of the "set digital output" command
        verbose_command = 1;
      case 'O': // "set digital output" command
        reqLength = 4;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandO(command_data[1], command_data[2], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'O' command
      case 'p': // verbose version of the "set PWM output" command
        verbose_command = 1;
      case 'P': // "set PWM output" command
        reqLength = 4;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandP(command_data[1], command_data[2], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'P' command
      case 'i': // verbose version of the "read digital input" command
        verbose_command = 1;
      case 'I': // "read digital input" command
        reqLength = 3;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandI(command_data[1], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'I' command
      case 'a': // verbose version of the "read analog input" command
        verbose_command = 1;
      case 'A': // "read analog input" command
        reqLength = 3;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandA(command_data[1], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'A' command
      case 'n': // verbose version of the "read counter" command
        verbose_command = 1;
      case 'N': // "read counter" command
        reqLength = 4;
        if ((command_data_length == reqLength)  && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandN(command_data[1], command_data[2], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'N' command
      case 1: // multi-set digital outputs
        reqLength = (2 * PINMASK_BYTESIZE + 2); //expected length of command
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          command1(); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 1 command
      case 2: // multi-read digital inputs
        reqLength = (PINMASK_BYTESIZE + 2); //expected length of command
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          command2(); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 2 command
      case 4: // process 16-bytes of user data
        reqLength = (18); //expected length of command
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          command4(); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 4 command

#ifdef USE1WIRE
      case 't': // verbose version of the "read 1-Wire temperature" command
        verbose_command = 1;
      case 'T': // "read 1-Wire temperature" command
        reqLength = 3;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandT(command_data[1], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'T' command
      case 3: // multi-read 1-Wire temperature
        reqLength = (PINMASK_BYTESIZE + 2); //expected length of command
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          command3(); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // command 3
#endif //USE1WIRE
#ifdef USEI2C
      case 'b': // verbose version of the "read Freescale MPL115A2 barometer" command
        verbose_command = 1;
      case 'B': // "read Freescale MPL115A2 barometer" command
        reqLength = 2;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandBarometer(verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'B' command
      case 'r': // verbose version of the "set DS1844 digital potentiometer wipers" command
        verbose_command = 1;
      case 'R': // "set DS1844 digital potentiometer wipers" command
        reqLength = 7;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandR(command_data[1], command_data[2], command_data[3], command_data[4], command_data[5], verbose_command); //execute the command
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'R' command
#endif //USEI2C
      case 'u': // verbose version of the "example user-defined function" command
        verbose_command = 1;
      case 'U': // "example user-defined function" command
        reqLength = 6;
        if ((command_data_length == reqLength) && (master_active == 1))
        { //correctly terminated command of the expected length received
#ifdef ENABLE_WDT
          wdt_reset();
#endif
          commandU(verbose_command); //execute the command, it will operate with the global array command_data
          command_data_length = 0;
        }
        else if (command_data_length > reqLength)
        { //command too long
          reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
          command_data_length = 0;
        }
        break; // 'U' command
      default: //unknown command received
        reportError(ERROR_COMMAND, ERROR_COMMAND_UNKNOWN);
        command_data_length = 0;
        break; //default case
    } //end switch command_data[0]

    if (Serial.available() > 60)
    { //master is sending too much data, report an error
      reportError(ERROR_SERIAL, ERROR_SERIAL_BUFFER_FULL);
    }
    else if (Serial.available() > 50)
    { //master is sending too much data, send a warning
      reportError(WARNING_SERIAL, WARNING_SERIAL_BUFFER_FULL);
    }
  } //end if lastIn
  else if (command_data_length == COMMAND_LENGTH_MAX)
  {
    reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
    command_data_length = 0;
  }
} //end loop
