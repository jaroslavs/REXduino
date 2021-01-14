/*********************************************************
 *                                                       *
    Functions for REXduino slave
    REXYGEN and Arduino communication over serial port
 *                                                       *
    Jaroslav SOBOTA, 2013-2021
    email: jsobota@ntis.zcu.cz
 *                                                       *
 *********************************************************/

// Convert an ASCII char to upper case ***************************************
byte uppercase(byte str) {
  if ((str >= 97) && (str <= 122)) {
    return (str - 32);
  }
  else {
    return (str);
  }
} //end uppercase

// Validate pin mode **********************************************************
byte validatePinMode(byte pin, byte mode) {
  //analog inputs should be referred to as 14..19 on Arduino UNO (A0..A5), 54..69 on MEGA (A0..A15), etc.
#ifdef USEI2C
  if ( (pin == SDA) || (pin == SCL) ) // 2 pins are reserved for I2C communication
  {
    pin = 255;
  }
#endif
  if ((pin >= 2) && (pin < NUM_DIGITAL_PINS)) //pins 0 and 1 used for serial communication
  {
    switch (mode) {
      case 'J':
        if (pin == 13)
        {
          pin = 255;
        }
        break;
      case 'W':
        if (pin == 13)
        {
          pin = 255;
        }
        break;
      case 'P':
        if (!digitalPinHasPWM(pin))
        {
          pin = 255;
        }
        break;
      case 'C':
      case 'E':
        if ((pin != 2) && (pin != 3))
        {
          pin = 255;
        }
        break;
      case 'B':
        if ((pin != 4) && (pin != 5))
        {
          pin = 255;
        }
        break;
      case 'A':
        if ((pin < A0) || (pin >= A0 + NUM_ANALOG_INPUTS))
        {
          pin = 255;
        }
    }
    if (pin == 255)
    {
      reportError(ERROR_PINMODE, ERROR_PINMODE_INVALID);
    }
    return pin;
  }
  else
  {
    reportError(ERROR_COMMAND, ERROR_COMMAND_PIN_NUMBER);
    return 255;
  }
}

// Report an error **********************************************************
void reportError(byte etype, byte ecode) {
  Serial.write('E');
  Serial.write(etype);
  Serial.write(ecode);
  Serial.write(';');
} //reportError

#ifdef USE1WIRE
// Print 1-Wire device ROM ***************************************************
void printOnewireROM(byte pin) {
  byte j;
  Serial.print("ROM= ");
  for ( j = 0; j < 8; j++) {
    Serial.print(onewire_deviceAddress[pin][j], HEX);
    Serial.print(" ");
  }
}

// Tell the master that there are no more 1-Wire devices on the branch
void reportOnewireDepleted(byte pin, byte verbose) {
  if (!verbose)
  {
    Serial.print("T");
    Serial.write(pin);
    Serial.write(STATUS_ONEWIRE_NOMOREDEVICES);
    Serial.print(';');
  }
}

// Prepare 1-Wire branch ****************************************************
void prepareOnewireBranch(byte pin) {
  onewire[pin]->reset();
  onewire[pin]->reset_search();
  onewire_deviceOrder[pin] = 0;
}

// Find next 1-Wire device ****************************************************
byte findOnewireDevice(byte pin, byte verbose) {
  if (onewire_deviceOrder[pin] < 255) { //the possibility of finding 1-Wire device still exists
    if (!onewire[pin]->search(&onewire_deviceAddress[pin][0])) { //no more 1-Wire devices found
      if (verbose) {
        Serial.print("1W END: p");
        Serial.print(pin);
        Serial.println("!");
      }
      onewire_deviceOrder[pin] = 255;
      onewire_deviceAddress[pin][0] = 0;
      return (STATUS_ONEWIRE_NOMOREDEVICES);
    }
    else // 1-Wire device found
    {
      onewire_deviceOrder[pin]++;
      if (verbose) {
        Serial.print("P");
        Serial.print(pin);
        Serial.print(", no. ");
        Serial.print(onewire_deviceOrder[pin]);
        Serial.print(": ");
        printOnewireROM(pin);
      }
      if ( OneWire::crc8(&onewire_deviceAddress[pin][0], 7) != onewire_deviceAddress[pin][7]) {
        if (verbose) {
          Serial.print("CRC inv!\n");
        }
        onewire_deviceAddress[pin][0] = 0;
        return (STATUS_ONEWIRE_INVALIDROM);
      }
      else {
        if (verbose) {
          Serial.print("OK!\n");
        }
        return (STATUS_ONEWIRE_SENSORFOUND);
      }
    }
  }
  else // this branch has already been completely depleted
  {
    if (verbose) {
      Serial.print("1W p");
      Serial.print(pin);
      Serial.print(" END.\n");
    }
    onewire_deviceAddress[pin][7] = 0;
    return (STATUS_ONEWIRE_DEPLETED);
  }
}

// Start temperature conversion on 1-Wire bus ****************************************************
byte convertOnewireTemp(byte pin, byte verbose) {
  if ((onewire_deviceAddress[pin][0] == 0x28) || (onewire_deviceAddress[pin][0] == 0x10) || (onewire_deviceAddress[pin][0] == 0x22)) { // 0x28=DS18B20, 0x10=DS18S20, 0x22=DS1822
    onewire[pin]->reset();
    onewire[pin]->select(&onewire_deviceAddress[pin][0]);
    onewire[pin]->write(0x44, 1); //initiate temperature conversion and leave the 1-Wire bus in HIGH state for powering parasitic devices
    if (verbose) {
      Serial.print("Measuring p");
      Serial.print(pin);
      Serial.print(" s");
      Serial.print(onewire_deviceOrder[pin]);
      Serial.print(": ");
      printOnewireROM(pin);
      Serial.println();
      Serial.print("1W p");
      Serial.print(pin);
      Serial.print(" block for");
      Serial.print(ONEWIRE_TEMPCONV_DELAY);
      Serial.println(" ms.");
    }
    return (1);
  }
  else {
    if (verbose) {
      Serial.println("No temp. sensor p");
      Serial.print(pin);
      Serial.print(", s");
      Serial.print(onewire_deviceOrder[pin]);
      Serial.print(" (type 0x");
      Serial.print(onewire_deviceAddress[pin][0], HEX);
      Serial.println(").");
    }
    return (0);
  }
}

// Read the result of 1-Wire temperature conversion ***************************************
void readOnewireTemp(byte pin, byte verbose) {
  byte i;
  byte data[12];
  if (verbose) {
    Serial.print("1W p");
    Serial.print(pin);
    Serial.println(" free");
    Serial.print("Read p");
    Serial.print(pin);
    Serial.print(" s");
    Serial.print(onewire_deviceOrder[pin]);
    Serial.print(": ");
    printOnewireROM(pin);
    Serial.println();
    Serial.print("HEX: ");
  }
  onewire[pin]->reset();
  onewire[pin]->select(&onewire_deviceAddress[pin][0]);
  onewire[pin]->write(0xBE, 1); //read scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = onewire[pin]->read();
    if (verbose) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    };
  }
  i = OneWire::crc8(data, 8);
  if (verbose) {
    Serial.print(" CRC=");
    Serial.println(i, HEX);
  };
  if (i == data[8]) { //CRC OK
    if (onewire_deviceAddress[pin][0] == 0x10) { // old DS18S20 (only 9 bit resolution by default, but improved precision available through the COUNT_REMAIN register, see datasheet)
      int temp_12bit;
      temp_12bit = (((data[1] << 8 | data[0]) << 3) & 0xFFF0) + 12 - data[6];
      data[1] = highByte(temp_12bit);
      data[0] = lowByte(temp_12bit);
    }
    else if ((onewire_deviceAddress[pin][0] == 0x28) || (onewire_deviceAddress[pin][0] == 0x22)) { //DS1822 or DS18B20
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00)  data[0] = data[0] & 0xF8;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) data[0] = data[0] & 0xFC; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) data[0] = data[0] & 0xFE; // 11 bit res, 375 ms
    }
    if (verbose) {
      Serial.print("Temp. p");
      Serial.print(pin);
      Serial.print(" s");
      Serial.print(onewire_deviceOrder[pin]);
      Serial.print(": ");
      Serial.print(((data[1] << 8) + data[0] ) * 0.0625, 4);
      Serial.println(" C.");
    }
    else {
      Serial.print("T");
      Serial.write(pin);
      Serial.write(STATUS_ONEWIRE_OK);
      Serial.write(onewire_deviceOrder[pin]);
      Serial.write(data[0]);
      Serial.write(data[1]);
      Serial.print(";");
    }
  }
  else { //CRC check failed
    if (verbose) {
      Serial.println("CRC inv!");
    }
    else {
      Serial.print("E");
      Serial.print(ERROR_ONEWIRE);
      Serial.write(ERROR_ONEWIRE_BADCRC);
    }
  }
}
#endif //USE1WIRE

/**
   Divides a given PWM pin frequency by a divisor.

   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.

   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2

   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.

   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4

   Arduino Uno pins 9 and 10
   Divisor	Frequency [Hz]
   1	31250
   8	3906.25
   64	488.28125
   256	122.0703125
  1024	30.517578125
*/
/*
  void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 64:
        mode = 0x03;
        break;
      case 256:
        mode = 0x04;
        break;
      case 1024:
        mode = 0x05;
        break;
      default:
        return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 32:
        mode = 0x03;
        break;
      case 64:
        mode = 0x04;
        break;
      case 128:
        mode = 0x05;
        break;
      case 256:
        mode = 0x06;
        break;
      case 1024:
        mode = 0x7;
        break;
      default:
        return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
  }
*/

// Counter on pin 2 ****************************************************
void ISRcounter2() {
  static byte old_C = digitalRead(2);
  static const char dir_lookup[] = {
    0, 0, -1, 1
  }; // use this line for counting falling edges
  // -1,1,0,0  }; // use this line for counting rising edges
  // -1,1,-1,1  }; // use this line for counting both edges
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > BUTTON_DEBOUNCE_MILLIS)
  {
    counter2 = counter2 + dir_lookup[(old_C << 1) | digitalRead(4)];
  }
  last_interrupt_time = interrupt_time;
  old_C = digitalRead(2);
}

// Counter on pin 3 ****************************************************
void ISRcounter3() {
  static byte old_C = digitalRead(3);
  static const char dir_lookup[] = {
    0, 0, -1, 1
  }; // use this line for counting falling edges
  // -1,1,0,0  }; // use this line for counting rising edges
  // -1,1,-1,1  }; // use this line for counting both edges
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > BUTTON_DEBOUNCE_MILLIS)
  {
    counter3 = counter3 + dir_lookup[(old_C << 1) | digitalRead(5)];
  }
  last_interrupt_time = interrupt_time;
  old_C = digitalRead(3);
}

// Encoder on pin 2 ****************************************************
void ISRencoder2() {
  static byte old_A = 0;
  static byte AB;
  static const char dir_lookup[] = {
    0, 0, 1, -1, -1, 1, 0, 0
  };
  AB = (digitalRead(2) << 1) | digitalRead(4);
  counter2 = counter2 + dir_lookup[(old_A << 2) | AB];
  old_A = (AB & 2) >> 1;
}

// Encoder on pin 3 ****************************************************
void ISRencoder3() {
  static byte old_A = 0;
  static byte AB;
  static byte enc_state;
  static const char dir_lookup[] = {
    0, 0, 1, -1, -1, 1, 0, 0
  };
  AB = (digitalRead(3) << 1) | digitalRead(5);
  counter3 = counter3 + dir_lookup[(old_A << 2) | AB];
  old_A = (AB & 2) >> 1;
}
