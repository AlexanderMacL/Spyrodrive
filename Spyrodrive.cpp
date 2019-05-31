#include <Arduino.h>
#undef min
#undef max
#include <Wire.h>
#include <SPI.h>
#include <stdlib.h>

#include "Spyrodrive.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "Adafruit_MCP23008.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

MODE op_mode = DISABLE_MODE;
K_PID pid_param = ENC;
SENS sensor_data = CAD;
int pressed=0;
char up_pressed=0, down_pressed=0, left_pressed=0, right_pressed=0, K_mode=SETPOINT_BUTTON;
float cad_buffer[CAD_BUF_LEN];

// A small helper
void error(const __FlashStringHelper*p_error) {
  Serial.println(p_error);
  while (1);
}

void initPWM(int freq) { // adapted from something I found online - think very carefully before changing PWMA_PIN!
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D5
  PORT->Group[g_APinDescription[PWMA_PIN].ulPort].PINCFG[g_APinDescription[PWMA_PIN].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D5 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[PWMA_PIN].ulPort].PMUX[g_APinDescription[PWMA_PIN].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = 24000000/freq;         // Set the frequency of the PWM on TCC0; 24 is 48/2, assume prescaler 1
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  // Set the PWM signal to output 0% duty cycle to begin with
  REG_TCC0_CC1 = 0;         // TCC0 CC1 - on D5
  while (TCC0->SYNCBUSY.bit.CC1);                // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void PWM_duty(byte duty) {
    // Set the PWM signal to output duty/255 duty cycle
  REG_TCC0_CC1 = REG_TCC0_PER*duty/255;         // TCC0 CC1 - on D5
  while (TCC0->SYNCBUSY.bit.CC1);                // Wait for synchronization
}

void initEncoder(Adafruit_MCP23008 * _mcp) {
  _mcp->begin();      // use default address 0
  _mcp->pinMode(0, INPUT);
  _mcp->pinMode(1, INPUT);
  _mcp->pinMode(2, INPUT);
  _mcp->pinMode(3, INPUT);
  _mcp->pinMode(4, INPUT);
  _mcp->pinMode(5, INPUT);
  _mcp->pinMode(6, INPUT);
  _mcp->pinMode(7, INPUT);
  _mcp->pullUp(0, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(1, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(2, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(3, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(4, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(5, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(6, HIGH);  // turn on a 100K pullup internally
  _mcp->pullUp(7, HIGH);  // turn on a 100K pullup internally
}

unsigned char getEncoderValue(Adafruit_MCP23008 * _mcp) {
  char val = _mcp->readGPIO();
  int i,j=5;
  do {
    for (i=0;i<128;i++) if (val==encoder_lookup[i]) break; // lookup from table
    i-=encoder_min_rad_value; // shift value to compensate for encoder start offset
    if (i<0) i+=128;
    if (i<128) return i;
    initEncoder(_mcp);
    val = _mcp->readGPIO();
  } while (--j>0);
  return 255; // error condition
}

unsigned char setPoint(float gear_ratio) { // returns index of ratio rounded up to next ratio
  unsigned char i;
  for (i=MIN_SETPOINT;i<MAX_SETPOINT;i++) if (gear_ratio<=ratio_lookup[i]) break; // lookup from table
  if (i>MAX_SETPOINT) return MAX_SETPOINT; // if desired ratio out of bounds
  else return i;
}

void initMotor(void) {
  initPWM(PWM_FREQ); // set up PWM
  pinMode(AIN1_PIN,OUTPUT);
  pinMode(AIN2_PIN,OUTPUT);
  digitalWrite(AIN1_PIN,0);
  digitalWrite(AIN1_PIN,1);
}

void spinMotor(unsigned char PWMvalue, unsigned char dir) { //PWMvalue 0-255, dir 0-1
  digitalWrite(AIN2_PIN,dir&0x1);
  digitalWrite(AIN1_PIN,(~dir)&0x1);
  PWM_duty(PWMvalue);
}

void spinMotor(signed char PWMvalue) {
  spinMotor(((PWMvalue==-128)?255:abs(PWMvalue)*2),((PWMvalue>0)?1:0));
}

void initBLE(Adafruit_BluefruitLE_SPI * BLEdevice) {
  if ( !BLEdevice->begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! BLEdevice->factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  BLEdevice->echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  BLEdevice->info();

  if (BLE_RESET_CHARS) {
    //arbitrarily choose 0x1900 as service ID
    BLEdevice->println("AT+GAPDEVNAME=Spyrodrive Final Build");
    BLEdevice->waitForOK();
    BLEdevice->println("AT+GATTCLEAR");
    BLEdevice->waitForOK();
    BLEdevice->println("AT+GATTADDSERVICE=UUID=0x1900");
    BLEdevice->readline_parseInt();
    BLEdevice->println("AT+GATTADDCHAR=UUID=0x0001,PROPERTIES=0x02,MIN_LEN=1,MAX_LEN=3,VALUE=000,DATATYPE=STRING,DESCRIPTION=EncoderPosition");
    char GATTCHAR1=BLEdevice->readline_parseInt();
    BLEdevice->println("ATZ");
    BLEdevice->waitForOK();
  }
}

/* getData()
 * reads data from BLE and/or Serial interfaces (if available) 
 * and configures mode switches depending on which buttons are pressed
 */
void getData(Adafruit_BluefruitLE_SPI * BLEdevice) {
  /* Wait for new data to arrive */
  uint8_t len = readPacket(BLEdevice, BLE_READPACKET_TIMEOUT);
  if (len > 1) {
    // Buttons
    if (packetbuffer[1] == 'B') {
      uint8_t buttnum = packetbuffer[2] - '0';
      pressed = packetbuffer[3] - '0';
      if (buttnum==UP_BUTTON) {
        if (pressed) up_pressed=1;
        else up_pressed=0;
      } else if (buttnum==DOWN_BUTTON) {
        if (pressed) down_pressed=1;
        else down_pressed=0;
      } else if (buttnum==LEFT_BUTTON) {
        if (pressed) left_pressed=1;
        else left_pressed=0;
      } else if (buttnum==RIGHT_BUTTON) {
        if (pressed) right_pressed=1;
        else right_pressed=0;
      } else if (buttnum==SETPOINT_BUTTON && !pressed) { // button 1 released
        K_mode=SETPOINT_BUTTON;
        verbose_mode=false;
        BLEdevice->println();
        switch (op_mode) { // cycles through operational modes
          case CADENCE_MODE:
            op_mode = SETPOINT_MODE;
            motor_enable = true;
            BLEdevice->print("RATIO MODE: ");
            BLEdevice->print((int)ratio);
            BLEdevice->println(" / 100 En");
            break;
          case SETPOINT_MODE:
            op_mode = DISABLE_MODE;
            motor_enable = false;
            BLEdevice->print("RATIO MODE: ");
            BLEdevice->print((int)ratio);
            BLEdevice->println(" / 100 Dis");
            break;
          case DISABLE_MODE:
            op_mode = CADENCE_MODE;
            motor_enable = true;
            BLEdevice->print("CADENCE MODE: set ");
            BLEdevice->print((int)cadence);
            BLEdevice->print(" | actual ");
            BLEdevice->println((int)actual_cadence);
            break;
        }
      } else if (buttnum==PID_BUTTON && pressed) {
        K_mode=PID_BUTTON;
        BLEdevice->println();
        verbose_mode=false;
        switch(pid_param) {
          case KP:
            pid_param=KI;
            break;
          case KI:
            pid_param=KD;
            break;
          case KD:
            pid_param=ENC;
            break;
          case ENC:
            pid_param=CKP;
            break;
          case CKP:
            pid_param=CKI;
            break;
          case CKI:
            pid_param=KP;
            break;
        }
      } else if (buttnum==DATA_BUTTON && pressed) {
        K_mode=DATA_BUTTON;
        verbose_mode=false;
        BLEdevice->println();
        if (!data_mode) {
          data_mode=true;
          sensor_data=GYRO;
          BLEdevice->println("Reporting Gyro data");
        } else {
          switch(sensor_data) {
            case GYRO:
              sensor_data=ACCEL;
              BLEdevice->println("Reporting Accel data");
              break;
            case ACCEL:
              sensor_data=MAG;
              BLEdevice->println("Reporting Mag data");
              break;
            case MAG:
              sensor_data=CAD;
              BLEdevice->println("Reporting Cadence data");
              break;
            case CAD:
              sensor_data=GYRO;
              data_mode=false;
              BLEdevice->println("Data Mode Disabled");
              break;
          }
        }
      } else if (buttnum==VERBOSE_BUTTON && pressed) {
        K_mode=VERBOSE_BUTTON;
        data_mode=false;
        verbose_mode=!verbose_mode;
        BLEdevice->println();
        verbose_mode?BLEdevice->println("Reporting System Data"):BLEdevice->println("Verbose Mode Disabled");
      }
    } else if (packetbuffer[1]=='A') {// could also have support for accelerometer data where tilting phone changes setpoint
      ratio+=(int)((*(float *)&packetbuffer[6])*10/9.81);
    }
  } else if (len==1) { // non-packet data
    int datcount=1;
    while ( BLEdevice->available() ) {
      packetbuffer[datcount++]=BLEdevice->read();
      packetbuffer[datcount]=0; // annul next element
    }
  } else {
    int datcount=0;
    while (Serial.available()) {
      packetbuffer[datcount++]=Serial.read();
      packetbuffer[datcount]=0; // annul next element
    }
  }
}

/* processData()
 * acts on any data which is not delivered as a packet (in the form ['!'][identifier][data][checksum]),
 * e.g. instructions from the Serial interface
 */
void processData(Adafruit_BluefruitLE_SPI * BLEdevice) {
  if (0==strcmp((char *)packetbuffer,"STOP")) {
    verbose_mode = data_mode = rand_ratio = false;
    staircase = 0;
  } else if (0==strcmp((char *)packetbuffer,"ACCEL") 
    || 0==strcmp((char *)packetbuffer,"GYRO") 
    || 0==strcmp((char *)packetbuffer,"MAG") 
    || 0==strcmp((char *)packetbuffer,"CADENCE")) {
    data_mode=true;
    verbose_mode=false;
    switch(packetbuffer[0]) {
      case 'A':
        sensor_data=ACCEL;
        if (ble_connected) BLEdevice->println("Reporting Accel data");
        break;
      case 'M':
        sensor_data=MAG;
        if (ble_connected) BLEdevice->println("Reporting Mag data");
        break;
      case 'G':
        sensor_data=GYRO;
        if (ble_connected) BLEdevice->println("Reporting Gyro data");
        break;
      case 'C':
        sensor_data=CAD;
        if (ble_connected) BLEdevice->println("Reporting Cadence data");
        break;
    }
  } else if (0==strcmp((char *)packetbuffer,"CONTROL")) {
    if (ble_connected) BLEdevice->println("Reporting control parameters");
    data_mode=false;
    verbose_mode=true;
  } else if (0==strcmp((char *)packetbuffer,"ENABLE")) motor_enable=true;
  else if (0==strcmp((char *)packetbuffer,"DISABLE")) motor_enable=false;
  else if (0==strcmp((char *)packetbuffer,"STEP")) {
    if (step_up) {
      ratio=90.0;
      step_up=false;
    }
    else {
      ratio=10.0;
      step_up=true;
    }
  } else if (0==strcmp((char *)packetbuffer,"RAND")) {
    timerstart=timestamp;
    rand_ratio=true;
  } else if (0==strcmp((char *)packetbuffer,"STAIRCASE")) {
    timerstart=timestamp;
    staircase = 10;
    step_up=!step_up;
    if (step_up) {
      ratio=10.0;
    }
    else {
      ratio=90.0;
    }
  } else if (0==strcmp((char *)packetbuffer,"+10")) { // increment of 10
    ratio+=10;
  } else if (0==strcmp((char *)packetbuffer,"-10")) { // decrement of 10
    ratio-=10;
  } else if (0==strcmp((char *)packetbuffer,"+1")) { // increment of 1
    ratio+=1;
  } else if (0==strcmp((char *)packetbuffer,"-1")) { // decrement of 1
    ratio-=1;
  }
}

/* updateButtons()
 * acts on Left/Right, Up/Down button presses in the context of the current mode switch settings
 * Left/Right decrease/increase the parameter selected in PID mode, and have no effect in VERBOSE and DATA modes 
 * Up/Down change the ratio in RATIO mode and the cadence in CADENCE mode
 */
void updateButtons(Adafruit_BluefruitLE_SPI * BLEdevice) {
  if (pressed == 1 && ble_connected) {
    switch(K_mode) {
      case SETPOINT_BUTTON: // button 1
        switch(op_mode) { // switches functions of up/down buttons
          case CADENCE_MODE:
            if (down_pressed || up_pressed) {
              cadence+=4*up_pressed;
              cadence-=4*down_pressed; 
            }
            break;
          case SETPOINT_MODE:
          case DISABLE_MODE:
            if (down_pressed || up_pressed) {
              ratio+=up_pressed;
              ratio-=down_pressed;
              // limit ratio so cannot accumulate beyond boundaries
              if (ratio>MAX_RATIO) ratio=MAX_RATIO;
              if (ratio<MIN_RATIO) ratio=MIN_RATIO;
              BLEdevice->println();
              BLEdevice->print("RATIO MODE: ");
              BLEdevice->print((int)ratio);
              BLEdevice->print(" / 100 ");
              BLEdevice->println(motor_enable?"En":"Dis");
            }
            break;
        }
        break;
      case PID_BUTTON:
        BLEdevice->println();
        switch(pid_param) {
          case KP:
            p_K_p+=(float)0.5*right_pressed;
            p_K_p-=(float)0.5*left_pressed;
            BLEdevice->print("p_K_p = ");
            BLEdevice->println(p_K_p);
            Serial.print("p_K_p = ");
            Serial.println(p_K_p);
            break;
          case KI:
            p_K_i+=(float)0.5*right_pressed;
            p_K_i-=(float)0.5*left_pressed;
            p_MAX_INTEGRAL = MAX_POWER/2/(p_K_i); // p_K_i cannot be zero
            BLEdevice->print("p_K_i = ");
            BLEdevice->println(p_K_i);
            Serial.print("p_K_i = ");
            Serial.println(p_K_i);
            break;
          case KD:
            p_K_d+=(float)0.5*right_pressed;
            p_K_d-=(float)0.5*left_pressed;
            BLEdevice->print("p_K_d = ");
            BLEdevice->println(p_K_d);
            Serial.print("p_K_d = ");
            Serial.println(p_K_d);
            break;
          case ENC:
            encoder_min_rad_value+=right_pressed;
            encoder_min_rad_value-=left_pressed;
            BLEdevice->print("EMRV = ");
            BLEdevice->println(encoder_min_rad_value);
            Serial.print("EMRV = ");
            Serial.println(encoder_min_rad_value);
            break;
          case CKP:
            c_K_p+=(float)0.01*right_pressed;
            c_K_p-=(float)0.01*left_pressed;
            BLEdevice->print("c_K_p = ");
            BLEdevice->println(c_K_p);
            Serial.print("c_K_p = ");
            Serial.println(c_K_p);
            break;
          case CKI:
            c_K_i+=(float)0.01*right_pressed;
            c_K_i-=(float)0.01*left_pressed;
            BLEdevice->print("c_K_i = ");
            BLEdevice->println(c_K_i);
            Serial.print("c_K_i = ");
            Serial.println(c_K_i);
            break;
        }
        break;
    }
  }/* else if (K_mode == PID_BUTTON && serial_connected) { // pressed = 0 due to else
    pressed = 0; // stops this happening continuously
    switch(pid_param) {
      case KP:
        Serial.print("p_K_p = ");
        Serial.println(p_K_p);
        break;
      case KI:
        Serial.print("p_K_i = ");
        Serial.println(p_K_i);
        break;
      case KD:
        Serial.print("p_K_d = ");
        Serial.println(p_K_d);
        break;
      case ENC:
        Serial.print("EMRV = "); // encoder min rad value
        Serial.println(encoder_min_rad_value);
        break;
    }
  }*/
  if (cadence<0) cadence=0;
  if (op_mode == CADENCE_MODE) {
    BLEdevice->println();
    BLEdevice->print("CADENCE MODE: set ");
    BLEdevice->print((int)cadence);
    BLEdevice->print(" | actual ");
    BLEdevice->println((int)actual_cadence);
  }
}

/*  outputData()
 *  reports requested data over the Serial/BLE interfaces (if active) in VERBOSE or DATA modes
 */
void outputData(Adafruit_BluefruitLE_SPI * BLEdevice, Adafruit_FXAS21002C * _gyro, Adafruit_FXOS8700 * _accelmag) {
  if (verbose_mode) {
    // control parameters required: timestamp,setpoint,measured,p_integral,p_derivative,output,enabled?,[ble_connected?]
    if (ble_connected) {
      BLEdevice->print(timestamp);BLEdevice->print(',');
      BLEdevice->print((int)cadence);BLEdevice->print(',');
      BLEdevice->print((int)actual_cadence);BLEdevice->print(',');
      BLEdevice->print(c_integral);BLEdevice->print(',');
      BLEdevice->print(setpoint);BLEdevice->print(',');
      BLEdevice->print(measured_value);BLEdevice->print(',');
      BLEdevice->print(p_integral);BLEdevice->print(',');
      BLEdevice->print(p_derivative);BLEdevice->print(',');
      BLEdevice->print(output);BLEdevice->print(',');
      BLEdevice->print(motor_enable);BLEdevice->print(',');
      BLEdevice->println(serial_connected);
    }
    if (serial_connected) {
      Serial.print(timestamp);Serial.print(',');
      Serial.print((int)cadence);Serial.print(',');
      Serial.print((int)actual_cadence);Serial.print(',');
      Serial.print(c_integral);Serial.print(',');
      Serial.print(setpoint);Serial.print(',');
      Serial.print(measured_value);Serial.print(',');
      Serial.print(p_integral);Serial.print(',');
      Serial.print(p_derivative);Serial.print(',');
      Serial.print(output);Serial.print(',');
      Serial.print(motor_enable);Serial.print(',');
      Serial.println(ble_connected);
    }
  } else if (data_mode) {
     sensors_event_t event;
     sensors_event_t aevent, mevent;
     BLEdevice->print(timestamp);BLEdevice->print(",");
    switch (sensor_data) {
      case GYRO:
        _gyro->getEvent(&event);
        if (serial_connected) {
          Serial.print(event.gyro.x*RAD_TO_DEG);Serial.print(",");
          Serial.print(event.gyro.y*RAD_TO_DEG);Serial.print(",");
          Serial.println(event.gyro.z*RAD_TO_DEG);
        }
        if (ble_connected) {
          BLEdevice->print(event.gyro.x*RAD_TO_DEG);BLEdevice->print(",");
          BLEdevice->print(event.gyro.y*RAD_TO_DEG);BLEdevice->print(",");
          BLEdevice->println(event.gyro.z*RAD_TO_DEG);
        }
        break;
      case ACCEL:
        _accelmag->getEvent(&aevent, &mevent);
        if (serial_connected) {
          Serial.print(aevent.acceleration.x);Serial.print(",");
          Serial.print(aevent.acceleration.y);Serial.print(",");
          Serial.println(aevent.acceleration.z);
        }
        if (ble_connected) {
          BLEdevice->print(aevent.acceleration.x);BLEdevice->print(",");
          BLEdevice->print(aevent.acceleration.y);BLEdevice->print(",");
          BLEdevice->println(aevent.acceleration.z);
        }
        break;
      case MAG:
        _accelmag->getEvent(&aevent, &mevent);
        if (serial_connected) {
          Serial.print(aevent.acceleration.x);Serial.print(",");
          Serial.print(aevent.acceleration.y);Serial.print(",");
          Serial.println(aevent.acceleration.z);
        }
        if (ble_connected) {
          BLEdevice->print(mevent.magnetic.x);BLEdevice->print(",");
          BLEdevice->print(mevent.magnetic.y);BLEdevice->print(",");
          BLEdevice->println(mevent.magnetic.z);  
        }
        break;
      case CAD:
        _gyro->getEvent(&event);
        if (serial_connected) {
          Serial.println(event.gyro.z*RAD_TO_DEG*GYRO_CORRECTION_FACTOR);
        }
        if (ble_connected) {
          BLEdevice->println(event.gyro.z*RAD_TO_DEG*GYRO_CORRECTION_FACTOR);
        }
        break;
    }
  }
}

float readCadence(Adafruit_FXAS21002C * _gyro) {
  sensors_event_t event;
  _gyro->getEvent(&event);
  return event.gyro.z*RAD_TO_DEG*GYRO_CORRECTION_FACTOR;
}

float * buf_push(float fl, float * buf, int buf_len) {
  if (!buf) return NULL;
  for (int i=buf_len-1;i>0;i--) {
    buf[i]=buf[i-1];
  }
  buf[0] = fl;
  return buf;
}

float mean(float * buf, int buf_len) {
  if (!buf) return NULL;
  float mn=0;
  for (int i=0;i<buf_len;i++) {
    mn+=buf[i];
  }
  return mn/buf_len;
}
