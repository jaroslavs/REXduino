/*********************************************************
 *                                                       *
    Commands for REXduino slave
    REXYGEN and Arduino communication over serial port
 *                                                       *
    Jaroslav SOBOTA, 2013-2020
    email: jsobota@ntis.zcu.cz
 *                                                       *
 *********************************************************/

// Perform C command - initialize communication ***************************************
void commandC(byte ver, byte verbose) {
  if (verbose) {
    ver = ver - 48; //convert from ASCII to number
  }
  if (ver == MAJORVERSION)
  {
    master_active = 1;
    Serial.print("C");
    Serial.write(48 + MAJORVERSION); //48 is the ASCII code of "0"
    Serial.print(";");
#ifdef ENABLE_WDT
    wdt_reset();
#endif
  }
  else
  {
    reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
  }
}

// Perform V command - version info ***************************************
void commandV(byte verbose) {
  if (verbose)
  {
    Serial.print("V. ");
    Serial.print(MAJORVERSION);
    Serial.print(".");
    Serial.print(MINORVERSION);
    Serial.print(".");
    Serial.print(REVISION);
    Serial.print(".");
    Serial.println(COMMIT);
  }
  else
  {
    Serial.print("V");
    Serial.write((byte)MAJORVERSION);
    Serial.write((byte)MINORVERSION);
    Serial.write((byte)REVISION);
    Serial.write((byte)COMMIT);
    Serial.print(";");
  }
}

// Perform B command - board info ***************************************
void commandB(byte verbose) {
  if (verbose)
  {
    Serial.print("Num.DI: ");
    Serial.println(NUM_DIGITAL_PINS);
    Serial.print("Num.AI: ");
    Serial.println(NUM_ANALOG_INPUTS);
    Serial.print("A0: ");
    Serial.println(A0);
    Serial.print("Lib: -");
#ifdef USE1WIRE
    Serial.print("W");
#endif
#ifdef USESERVO
    Serial.print("S");
#endif
#ifdef USEI2C
    Serial.print("I");
#endif
    Serial.println();
  }
  else
  {
    // N/A
  }
}

// Perform M command - change pin mode ***************************************
void commandM(byte pin, byte mode, byte verbose) {
  //in M2M communication, analog inputs should be referred to as 14..19 on Arduino UNO (A0..A5), 54..69 on MEGA (A0..A15), etc.
  //in verbose mode, analog inputs are numbered from 0
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
    pin = A0 + pin; //e.g. 14+pin on Arduino UNO, i.e. 0..5 -> 14..19
    mode = uppercase(mode);
  }
  switch (mode) { //set the pin to the desired mode
    case 'N': //not connected
    case 'A': //analog input
    case 'I': //input
    case 'B': //encoder signal B or counter DIR
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        pinMode(pin, INPUT);
      }
      break;
    case 'J': //input with pullup
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        pinMode(pin, INPUT_PULLUP);
      }
      break;
    case 'O': //output, default LOW
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
      }
      break;
    case 'Q': //output, default HIGH
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
      }
      break;
    case 'C': //counter
      if (pin == 2)
      {
        pinMode(2, INPUT_PULLUP);
        counter2 = 0;
        CNT2EN = CNT2EN_old = 0;
        attachInterrupt(0, ISRcounter2, CHANGE); //pulses on pin 2, direction signal on pin 4
      }
      if (pin == 3)
      {
        pinMode(3, INPUT_PULLUP);
        counter3 = 0;
        CNT3EN = CNT3EN_old = 0;
        attachInterrupt(1, ISRcounter3, CHANGE); //pulses on pin 3, direction signal on pin 5
      }
      break;
    case 'E': //encoder
      if (pin == 2)
      {
        pinMode(2, INPUT);
        counter2 = 0;
        CNT2EN = CNT2EN_old = 0;
        attachInterrupt(0, ISRencoder2, CHANGE); //encoder signal A on pin 2, signal B on pin 4
      }
      if (pin == 3)
      {
        pinMode(3, INPUT);
        counter3 = 0;
        CNT3EN = CNT3EN_old = 0;
        attachInterrupt(1, ISRencoder3, CHANGE); //encoder signal A on pin 3, signal B on pin 5
      }
      break;
#ifdef USE1WIRE
    case 'W': //1-wire branch
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        prepareOnewireBranch(pin); //reset 1-Wire bus
      }
      break;
#endif
    case 'P': //analog output (PWM)
      pin = validatePinMode(pin, mode);
      if (pin < 255)
      {
        // no action necessary
      }
      break;
    default: //unknown pinmode
      reportError(ERROR_PINMODE, ERROR_PINMODE_UNKNOWN);
      pin = 255;
      break;
  } //end switch(mode)
  if  (pin < 255) { //valid pin number and pin mode, send command response
    if (verbose) {
      Serial.println();
      Serial.println("Init:");
      Serial.print("m");
      Serial.print(pin);
      Serial.write(mode);
      Serial.println();
    }
    else {
      Serial.write('M');
      Serial.write(pin);
      Serial.write(mode);
      Serial.write(';');
    }  //end if(verbose)
  } //end if(pin<255)
} // end command M

// Perform O command - set digital output ***************************************
void commandO(byte pin, byte state, byte verbose) {
  if (verbose)
  {
    pin = pin - 48; //convert from ASCII to number
    state = state - 48; //convert from ASCII to number
  }
  pin = validatePinMode(pin, 'O');
  if  (pin < 255) //valid pin number and state of the output
  {
    switch (state)
    { //set the pin to the desired mode
      case 0:
        digitalWrite(pin, LOW);
        break;
      case 1:
        digitalWrite(pin, HIGH);
        break;
      default:
        reportError(ERROR_COMMAND, ERROR_COMMAND_INVALID);
        pin = 255;
        break;
    }
  }
  if  (pin < 255) //valid pin state
  {
    if (verbose)
    {
      Serial.println();
      Serial.println("DO:");
      Serial.print("p");
      Serial.print(pin);
      Serial.print(" s");
      Serial.print(state);
      Serial.println();
    }
    else {
      Serial.write('O');
      Serial.write(pin);
      Serial.write(state);
      Serial.write(';');
    }  //end if(verbose)
  } //end if(pin<255)
}

// Perform P command - set PWM output ***************************************
void commandP(byte pin, byte value, byte verbose) {
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
  }
  pin = validatePinMode(pin, 'P');
  if  (pin < 255) //valid pin number
  {
    analogWrite(pin, value);
    if (verbose) {
      Serial.println();
      Serial.println("PWM:");
      Serial.print("p");
      Serial.print(pin);
      Serial.print(" v");
      Serial.print(value);
      Serial.println();
    }
    else {
      Serial.write('P');
      Serial.write(pin);
      Serial.write(value);
      Serial.write(';');
    }  //end if(verbose)
  } //end if(pin<255)
}

// Perform I command - read digital input ***************************************
void commandI(byte pin, byte verbose) {
  byte state;
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
  }
  pin = validatePinMode(pin, 'I');
  if  (pin < 255) //valid pin number
  {
    state = digitalRead(pin);
    if (verbose) {
      Serial.println();
      Serial.println("DI:");
      Serial.print("p");
      Serial.print(pin);
      Serial.print(" s");
      Serial.print(state);
      Serial.println();
    }
    else {
      Serial.write('I');
      Serial.write(pin);
      Serial.write(state);
      Serial.write(';');
    }  //end if(verbose)
  } //end if(pin<255)
}

// Perform A command - read analog input ***************************************
void commandA(byte pin, byte verbose) {
  //in M2M communication, analog inputs should be referred to as 14..19 on Arduino UNO (A0..A5), 54..69 on MEGA (A0..A15), etc.
  //in verbose mode, analog inputs are numbered from 0
  int value;
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
    pin = A0 + pin; //e.g. 14+pin on Arduino UNO, i.e. 0..5 -> 14..19
  }
  pin = validatePinMode(pin, 'A');
  if  (pin < 255) //valid pin number
  {
    value = analogRead(pin);
    if (verbose) {
      Serial.println();
      Serial.println("AI:");
      Serial.print("p");
      Serial.print(pin);
      Serial.print(" v");
      Serial.print(value);
      Serial.println();
    }
    else {
      Serial.write('A');
      Serial.write(pin);
      Serial.write(highByte(value));
      Serial.write(lowByte(value));
      Serial.write(';');
    }  //end if(verbose)
  } //end if(pin<255)
}

// Perform N command - counter ***************************************
void commandN(byte pin, byte statusByte, byte verbose) {
  long counter, command_type;
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
    statusByte = statusByte - 48; //convert from ASCII to number
  }
  pin = validatePinMode(pin, 'N');
  if  (pin < 255) //valid pin number
  {
    if (verbose)
    {
      Serial.println();
      Serial.print("CNT");
      Serial.print(pin);
      Serial.print(": ");
    }
    if ((statusByte & 2) == 2)
    {
      command_type = COMMAND_CNT_RESET;
    }
    else if ((statusByte & 1) == 1)
    {
      command_type = COMMAND_CNT_ENABLE;
    }
    else if ((statusByte & 1) == 0)
    {
      command_type = COMMAND_CNT_DISABLE;
    }
    switch (command_type)
    {
      case COMMAND_CNT_RESET:
        if (pin == 2)
        {
          counter2 = 0;
          CNT2hld = 0;
        }
        else if (pin == 3)
        {
          counter3 = 0;
          CNT3hld = 0;
        }
        if (verbose)
        {
          Serial.println("RES");
        }
        break;
      case COMMAND_CNT_ENABLE:
        if (pin == 2)
        {
          if (CNT2EN_old == 0)
          {
            counter2 = CNT2hld;
          }
          CNT2EN = CNT2EN_old = 1;
        }
        else if (pin == 3)
        {
          if (CNT3EN_old == 0)
          {
            counter3 = CNT3hld;
          }
          CNT3EN = CNT3EN_old = 1;
        }
        if (verbose)
        {
          Serial.println("EN");
        }
        break;
      case COMMAND_CNT_DISABLE:
        if (pin == 2)
        {
          if (CNT2EN_old == 1)
          {
            CNT2hld = counter2;
          }
          CNT2EN = CNT2EN_old = 0;
        }
        else if (pin == 3)
        {
          if (CNT3EN_old == 1)
          {
            CNT3hld = counter3;
          }
          CNT3EN = CNT3EN_old = 0;
        }
        if (verbose)
        {
          Serial.println("DIS");
        }
        break;
    } //end switch
    if (pin == 2)
    {
      if (CNT2EN)
      {
        counter = counter2;
      }
      else
      {
        counter = CNT2hld;
      }
      statusByte = CNT2EN;
    }
    else if (pin == 3)
    {
      if (CNT3EN)
      {
        counter = counter3;
      }
      else
      {
        counter = CNT3hld;
      }
      statusByte = CNT3EN;
    }
    if (verbose)
    {
      Serial.println(counter);
    }
    else
    {
      Serial.write('N');
      Serial.write(pin);
      Serial.write(statusByte);
      Serial.write(counter & 255);
      Serial.write((counter >> 8) & 255);
      Serial.write((counter >> 16) & 255);
      Serial.write((counter >> 24) & 255);
      Serial.write(";");
    }
  } //end if(pin<255)
}

// Perform command 1 - multi-set digital outputs ***************************************
void command1(void) {
  byte i, j;
  for (i = 0; i < PINMASK_BYTESIZE; i++)
  {
    for (j = 0; j < 8; j++)
    {
      if ( command_data[i + 1] & (1 << j) )
      {
        commandO(i * 8 + j, (command_data[PINMASK_BYTESIZE + i + 1] & (1 << j)) >> j, 0);
      }
    }
  }
} // end of command 1

// Perform command 2 - multi-read digital inputs ***************************************
void command2(void) {
  byte i, j;
  for (i = 0; i < PINMASK_BYTESIZE; i++)
  {
    for (j = 0; j < 8; j++)
    {
      if ( command_data[i + 1] & (1 << j) )
      {
        commandI(i * 8 + j, 0);
      }
    }
  }
} // end of command 2

// Perform command 4 - process 16-bytes of user data ***************************************
void command4(void) {
  byte i;
  //user data is in global array command_data, LSB is in command_data[1], MSB is in command_data[16]
  //process the incoming data here
  //...
  //compose response - by default the data will be looped back
  Serial.write(4);
  for (i = 0; i < 16; i++)
  {
    Serial.write(command_data[i + 1]);
  }
  Serial.write(';');
} // end of command 4

#ifdef USE1WIRE
// Perform T command - read 1-Wire temperature ***************************************
void commandT(byte pin, byte verbose) {
  if (verbose) {
    pin = pin - 48; //convert from ASCII to number
  }
  pin = validatePinMode(pin, 'W');
  if  (pin < 255) //valid pin number
  {
    if (onewire_busBlockingLength[pin] > 0)
    { //some device needs the bus to remain in HIGH state
      if (verbose)
      {
        Serial.println("1W busy!");
      }
      reportError(ERROR_ONEWIRE, ERROR_ONEWIRE_BUSYBUS);
    }
    else
    { //1-Wire bus is available
      switch (findOnewireDevice(pin, verbose))
      {
        case STATUS_ONEWIRE_DEPLETED: //not necessarily an error
        //      reportError(ERROR_ONEWIRE,ERROR_ONEWIRE_DEPLETED);
        //      break;
        case STATUS_ONEWIRE_NOMOREDEVICES:
          onewire_busBlockingStart[pin] = millis();
          onewire_busBlockingLength[pin] = 100;
          if (verbose)
          {
            onewire_postponedAction[pin] = ACTION_POSTPONE_REPORT_DEPLETED_VERBOSE;
          }
          else
          {
            onewire_postponedAction[pin] = ACTION_POSTPONE_REPORT_DEPLETED;
          }
          /*
            Serial.print("T");
            Serial.write(pin);
            Serial.write(STATUS_ONEWIRE_NOMOREDEVICES);
            Serial.print(';');
          */
          break;
        case STATUS_ONEWIRE_INVALIDROM:
          reportError(ERROR_ONEWIRE, ERROR_ONEWIRE_BADCRC);
          break;
        case STATUS_ONEWIRE_SENSORFOUND:
          if (convertOnewireTemp(pin, verbose)) { //temperature conversion successfully started
            onewire_busBlockingStart[pin] = millis();
            onewire_busBlockingLength[pin] = ONEWIRE_TEMPCONV_DELAY;
            if (verbose)
            {
              onewire_postponedAction[pin] = ACTION_POSTPONE_OWTEMP_VERBOSE;
            }
            else
            {
              onewire_postponedAction[pin] = ACTION_POSTPONE_OWTEMP;
              Serial.print("T");
              Serial.write(pin);
              Serial.write(STATUS_ONEWIRE_TEMPCONV);
              Serial.print(';');
            }
          }
          else { //temperature conversion was not started
            reportError(ERROR_ONEWIRE, ERROR_ONEWIRE_NOTEMPDEVICE);
          }
          break;
      } //end switch
    } //end "bus available"
  } //end if pin<255
}

// Perform command 3 - multi-read 1-Wire temperature ***************************************
void command3(void) {
  byte i, j;
  for (i = 0; i < PINMASK_BYTESIZE; i++)
  {
    for (j = 0; j < 8; j++)
    {
      if ( command_data[i + 1] & (1 << j) )
      {
        commandT(i * 8 + j, 0);
      }
    }
  }
} //end of command 3
#endif //USE1WIRE

#ifdef USEI2C
// Perform B command - read MPL115A2 barometer ***************************************
void commandBarometer(byte verbose) {
  byte i = 0;
  byte data[12];
  byte result = 0;

  if (verbose) Serial.println("MPL115A2: ");
  Wire.beginTransmission(0x60); // talk to MPL115A2
  Wire.write(0x12);            // CONVERT command
  Wire.write(0x00);            // necessary complement
  result = Wire.endTransmission();     // send the buffered data to I2C slave
  if (result) {
    if (verbose) Serial.println("N/A");
    reportError(ERROR_I2C, ERROR_I2C_BAROMETER);
    return;
  }
  delay(4); //let the conversions finish...
  Wire.beginTransmission(0x60); // talk to MPL115A2
  Wire.write(0x00);            // "read pressure MSB" command
  result = Wire.endTransmission();     // stop transmitting
  if (result) {
    if (verbose) Serial.println("N/A");
    reportError(ERROR_I2C, ERROR_I2C_BAROMETER);
    return;
  }
  Wire.requestFrom(0x60, 12); //read the whole device memory (12 bytes)
  while (Wire.available())   // barometer might send less than requested
  {
    data[i] = Wire.read(); // read data
    i++;
  }
  if (i == 12) { //data successfully read from the barometer
    if (verbose) {
      Serial.print("HEX data: ");
      for (i = 0; i < 12; i++) {
        Serial.print(data[i], HEX);         // print byte
        Serial.print(' ');
      }
      Serial.println();
    }
    else {
      Serial.print('B');
      for (i = 0; i < 12; i++) {
        Serial.write(data[i]);         // send data to host
      }
      Serial.print(';');
    }
  }
  else reportError(ERROR_I2C, ERROR_I2C_BAROMETERDATA); //wrong number of bytes received from the barometer
}

// Perform R command - set DS1844 digital potentiometer ***************************************
void commandR(byte addr, byte pot0, byte pot1, byte pot2, byte pot3, byte verbose) {
  byte result = 0;
  if (verbose) {
    addr = addr - 48; //convert from ASCII to number
    Serial.println("DS1844: ");
  }
  if ((pot0 > 63) || (pot0 > 63) || (pot0 > 63) || (pot0 > 63)) {
    if (verbose) {
      Serial.println("pot>63");
    }
    reportError(ERROR_I2C, ERROR_I2C_RESISTORDATA);
    return;
  }
  Wire.beginTransmission(addr + 0x28); // talk to DS1844, addr is binary combination of "A2 A1 A0" (e.g. 3 for A2=LOW, A1=A0=HIGH)
  Wire.write(pot0);            // value of pot0 (6-bit)
  Wire.write(64 + pot1);          // selection of pot1 plus its value
  Wire.write(128 + pot2);          // selection of pot2 plus its value
  Wire.write(192 + pot3);          // selection of pot3 plus its value
  result = Wire.endTransmission();     // send the buffered data to I2C slave
  if (result) { //problem talking to ds1844
    if (verbose) {
      Serial.print("N/A (addr. ");
      Serial.print(addr);
      Serial.println(")");
    }
    reportError(ERROR_I2C, ERROR_I2C_RESISTOR);
  }
  else { //talking to DS1844 OK
    if (verbose) {
      Serial.print("DS1844 OK: ");
      Serial.print(pot0);
      Serial.print(' ');
      Serial.print(pot1);
      Serial.print(' ');
      Serial.print(pot2);
      Serial.print(' ');
      Serial.println(pot3);
    }
    else { //response to host
      Serial.write("R");
      Serial.write(addr);
      Serial.write(";");
    }
  }
}
#endif //USEI2C

// Perform U command - an example of user-defined function ***************************************
// returns uptime in miliseconds or microseconds
void commandU(byte verbose) {
  if (verbose) {
    command_data[1] = (command_data[1] & 0xFF) - 48; //convert from ASCII to number
  }
  if (verbose) {
    Serial.println();
    if (command_data[1] == 0) {
      Serial.print("Uptime [ms]: ");
      Serial.println(millis());
    }
    else {
      Serial.print("Uptime [us]: ");
      Serial.println(micros());
    }
  }
  else {
    Serial.write('U');
    if (command_data[1] == 0) {
      Serial.write((millis() & 255));
      Serial.write((millis() >> 8) & 255);
      Serial.write((millis() >> 16) & 255);
      Serial.write((millis() >> 24) & 255);
    }
    else {
      Serial.write((micros() & 255));
      Serial.write((micros() >> 8) & 255);
      Serial.write((micros() >> 16) & 255);
      Serial.write((micros() >> 24) & 255);
    }
    Serial.write(";");
  }
}
