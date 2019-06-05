/* 
 *  Spyrodrive Final Build GET EXCITED
 *  functionality added from testBench2:
 0  extra PI control loop added for cadence optimisation
 0  cadence measurement and calibration from gyro values - check out whether the thing that is wrong is that the range
 *  
 */

 /* Still to do:
  *  Reduce refresh rate in CADENCE mode so BLE never overloaded
  */

#include <Arduino.h>
#undef min
#undef max
#include <Wire.h>
#include <SPI.h>
#include <stdlib.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "Adafruit_MCP23008.h"

#include "Spyrodrive.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_MCP23008 mcp;

// Assign unique ID to sensors
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void setup() {
  // put your setup code here, to run once:
  initEncoder(&mcp);
  initMotor();
  Serial.begin(115200);  // start serial for output
  initBLE(&ble);
  ble.setMode(BLUEFRUIT_MODE_DATA);

  if(!gyro.begin(GYRO_RANGE_500DPS)) {Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }
  if(!accelmag.begin(ACCEL_RANGE_4G)) {Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
}

// variables for discretiastion of PI
float c_olderror = 0;
float c_integral = 0;
float c_error;
float c_K_p = 0.18;
float c_K_i = 0.04;
float cadence=40; // cadence defaults to 40 rpm
float c_MAX_INTEGRAL = cadence/10/(c_K_i);
float actual_cadence; // cadence setpoint as opposed to the actual cadence

// variables for discretisation of PID
float p_olderror = 0;
float p_integral = 0;
float p_error,p_derivative;
int setpoint=MIN_SETPOINT,measured_value,output;
int encoder_min_rad_value=42;

// parameters for PID control
const int output_thresh = 40; // minimum motor voltage threshold
float p_K_p = 30;
float p_K_i = 5;
float p_K_d = 1.7;
float p_MAX_INTEGRAL = MAX_POWER/2/(p_K_i);
float p_MAX_DERIVATIVE = 100;
float ratio=2; // set default ratio to 1% of total change

bool motor_enable = false; // motor disabled on powerup
bool verbose_mode = false; // parameter reporting over Serial disabled on powerup
bool data_mode = false; // sensor data collection and reporting over Serial disabled on powerup
bool ble_connected = false;
bool serial_connected = false;
bool step_up=false;
bool rand_ratio = false;

unsigned long timestamp=0, oldtimestamp, timerstart;
int staircase=0;

void loop() {
  // put your main code here, to run repeatedly:
  ble_connected = ble.isConnected();
  serial_connected = Serial?true:false;
  
  getData(&ble); // read BLE/Serial interfaces and configure mode switches
  processData(&ble); // process non-packet data
  updateButtons(&ble); // service button packets in the context of mode switches

  if (rand_ratio) { // random steps every 5s for testing purposes
    if ((timestamp-timerstart) > 5000) {
      ratio = random(10,90);
      timerstart=timestamp;
    }
  }
  
  if (staircase) { // function intended for testing - creates a 'staircase' of setpoints to test system response at different radii
    if ( (timestamp-timerstart) > ((staircase==10)?10000:5000) ) { // delay for 10 seconds before 1st of 5 steps and for 5 seconds between steps
      timerstart=timestamp; // reset timer
      if (step_up) ratio=5+10*(10-(staircase--)); // set new ratio (upward steps)
      else ratio=5+10*(--staircase); // (downward steps)r
    }
  }

  oldtimestamp=timestamp;
  timestamp=millis(); // rolls over every 49.7 days
  // PI control loop
  actual_cadence = mean(buf_push(readCadence(&gyro),cad_buffer,CAD_BUF_LEN),CAD_BUF_LEN);
  if (actual_cadence<0) actual_cadence=0;
  c_error = cadence - actual_cadence;

  c_integral = c_integral + c_error * (timestamp-oldtimestamp)/1000;
  if (c_error==0 || (c_error/abs(c_error))!=(c_olderror/abs(c_olderror))) c_integral=0;// chop the integral at zero error or if error changes sign
  if (c_integral>c_MAX_INTEGRAL) c_integral=c_MAX_INTEGRAL;
  if (c_integral<-1*c_MAX_INTEGRAL) c_integral=-1*c_MAX_INTEGRAL;

  if (op_mode != CADENCE_MODE) { // negate effect of PI loop on ratio if not in cadence control mode
    c_error=0;
    c_integral=0;
  }
  ratio -= (int)(c_K_p*c_error + c_K_i*c_integral);
  c_olderror=c_error;

  // limit ratio so cannot accumulate beyond boundaries
  if (ratio>MAX_RATIO) ratio=MAX_RATIO;
  if (ratio<MIN_RATIO) ratio=MIN_RATIO;
  
  // PID control loop

  setpoint=setPoint(ratio); // look up setpoint from ratio table (will keep setpoint bounded)

  measured_value = getEncoderValue(&mcp);
  p_error = setpoint - measured_value;

  p_integral = p_integral + p_error * (timestamp-oldtimestamp)/1000;
  if (p_error==0 || (p_error/abs(p_error))!=(p_olderror/abs(p_olderror))) p_integral=0;// chop the integral at zero error or if error changes sign

  if (p_integral>p_MAX_INTEGRAL) p_integral=p_MAX_INTEGRAL;
  if (p_integral<-1*p_MAX_INTEGRAL) p_integral=-1*p_MAX_INTEGRAL;
  p_derivative = (p_error-p_olderror)*1000/(timestamp-oldtimestamp);
  if (p_derivative>p_MAX_DERIVATIVE) p_derivative=p_MAX_DERIVATIVE;
  if (p_derivative<-1*p_MAX_DERIVATIVE) p_derivative=-1*p_MAX_DERIVATIVE;
  output = (int)(p_K_p*p_error + p_K_i*p_integral + p_K_d*p_derivative);
  p_olderror = p_error;

  // prevent output of a small value
  if (abs(output)<output_thresh) {
    if (abs(output)>output_thresh/2) output=output_thresh*((output<0)?-1:1); // limit madnitude of output
    else output=0;
  }

  // limit output
  if (output>MAX_POWER) output=MAX_POWER;
  if (output<-128) output=-1*(MAX_POWER+1);
  if (motor_enable) spinMotor(output);
  else spinMotor(0);

  outputData(&ble,&gyro,&accelmag); // write data over Serial/BLE if in VERBOSE or DATA mode
  
  //delay(100);
  

}
