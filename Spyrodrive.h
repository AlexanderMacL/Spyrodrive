#ifndef SPYRODRIVE_H
#define SPYRODRIVE_H

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "Adafruit_MCP23008.h"

#define FACTORYRESET_ENABLE   1
#define BLE_RESET_CHARS       1

// PWMA on pin 5
#define PWMA_PIN 5
#define AIN1_PIN 10
#define AIN2_PIN 9
#define PWM_FREQ 50000

#define SETPOINT_BUTTON 1
#define PID_BUTTON 2
#define DATA_BUTTON 3
#define VERBOSE_BUTTON 4

#define UP_BUTTON 5
#define DOWN_BUTTON 6
#define LEFT_BUTTON 7
#define RIGHT_BUTTON 8

#define MIN_SETPOINT 3
#define MAX_SETPOINT 117

#define MIN_RATIO 0.0
#define MAX_RATIO 100.0

#define MAX_POWER 127

#define GYRO_CORRECTION_FACTOR -0.667
#define CAD_BUF_LEN 10

// the packet buffer
extern uint8_t packetbuffer[];
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);

void initPWM(int freq);
void PWM_duty(byte duty);
void initEncoder(Adafruit_MCP23008 * _mcp);
unsigned char getEncoderValue(Adafruit_MCP23008 * _mcp);
unsigned char setPoint(float gear_ratio);
void initMotor(void);
void spinMotor(unsigned char PWMvalue, unsigned char dir);
void spinMotor(signed char PWMvalue);
void initBLE(Adafruit_BluefruitLE_SPI * BLEdevice);
void getData(Adafruit_BluefruitLE_SPI * BLEdevice);
void processData(Adafruit_BluefruitLE_SPI * BLEdevice);
void updateButtons(Adafruit_BluefruitLE_SPI * BLEdevice);
void outputData(Adafruit_BluefruitLE_SPI * BLEdevice, Adafruit_FXAS21002C * _gyro, Adafruit_FXOS8700 * _accelmag);
float readCadence(Adafruit_FXAS21002C * _gyro);
float * buf_push(float fl, float * buf, int buf_len);
float mean(float * buf, int buf_len);

enum MODE {CADENCE_MODE,SETPOINT_MODE,DISABLE_MODE};
extern MODE op_mode;
enum K_PID {KP,KI,KD,ENC,CKP,CKI};
extern K_PID pid_param;
enum SENS {ACCEL,GYRO,MAG,CAD};
extern SENS sensor_data;
extern int encoder_min_rad_value;
extern int pressed;
extern bool motor_enable;
extern bool verbose_mode;
extern bool data_mode;
extern bool ble_connected;
extern bool serial_connected;
extern bool step_up;
extern bool rand_ratio;
extern float ratio;
extern int staircase;
extern unsigned long timestamp, timerstart;
extern float p_K_p,p_K_i,p_K_d,p_MAX_INTEGRAL;
extern float p_integral;
extern float p_error,p_derivative;
extern int setpoint,measured_value,output;
extern float c_integral, c_error;
extern float c_K_p, c_K_i, c_MAX_INTEGRAL;
extern float cadence,actual_cadence; // this is the cadence setpoint as opposed to the actual cadence
extern float cad_buffer[];

const char encoder_lookup[128] = {127,63,62,58,56,184,152,24,
                                  8,72,73,77,79,15,47,175,
                                  191,159,31,29,28,92,76,12,
                                  4,36,164,166,167,135,151,215,
                                  223,207,143,142,14,46,38,6,
                                  2,18,82,83,211,195,203,235,
                                  239,231,199,71,7,23,19,3,
                                  1,9,41,169,233,225,229,245,
                                  247,243,227,163,131,139,137,129,
                                  128,132,148,212,244,240,242,250,
                                  251,249,241,209,193,197,196,192,
                                  64,66,74,106,122,120,121,125,
                                  253,252,248,232,224,226,98,96,
                                  32,33,37,53,61,60,188,190,
                                  254,126,124,116,112,113,49,48,
                                  16,144,146,154,158,30,94,95
};

const float ratio_lookup[121] = {0, 0.521276596, 1.047281324, 1.576832151, 2.109929078,
                                2.647754137, 3.189125296, 3.734042553, 4.283687943, 4.836879433,
                                5.393617021, 5.955082742, 6.521276596, 7.091016548, 7.665484634,
                                8.243498818, 8.825059102, 9.412529551, 10.0035461, 10.59929078,
                                11.19858156, 11.80260047, 12.41134752, 13.0248227, 13.64184397,
                                14.26477541, 14.89125296, 15.52245863, 16.15839243, 16.79905437,
                                17.44444444, 18.09456265, 18.74940898, 19.40898345, 20.07328605,
                                20.74468085, 21.41843972, 22.09219858, 22.77777778, 23.4751773,
                                24.1607565, 24.86997636, 25.56737589, 26.27659574, 26.99763593,
                                27.71867612, 28.43971631, 29.17257683, 29.90543735, 30.6501182,
                                31.39479905, 32.15130024, 32.90780142, 33.67612293, 34.44444444,
                                35.21276596, 35.9929078, 36.78486998, 37.57683215, 38.38061466,
                                39.18439716, 39.98817967, 40.80378251, 41.63120567, 42.45862884,
                                43.28605201, 44.12529551, 44.97635934, 45.82742317, 46.69030733,
                                47.55319149, 48.42789598, 49.30260047, 50.1891253, 51.07565012,
                                51.97399527, 52.88416076, 53.79432624, 54.70449173, 55.63829787,
                                56.56028369, 57.50591017, 58.45153664, 59.39716312, 60.35460993,
                                61.32387707, 62.29314421, 63.27423168, 64.26713948, 65.26004728,
                                66.26477541, 67.26950355, 68.28605201, 69.3144208, 70.3427896,
                                71.38297872, 72.43498818, 73.48699764, 74.55082742, 75.61465721,
                                76.69030733, 77.77777778, 78.87706856, 79.97635934, 81.08747045,
                                82.21040189, 83.33333333, 84.46808511, 85.61465721, 86.76122931,
                                87.91962175, 89.08983452, 90.27186761, 91.45390071, 92.64775414,
                                93.8534279, 95.05910165, 96.28841608, 97.5177305, 98.75886525, 100};

#endif
