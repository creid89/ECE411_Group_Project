/*
 * 1. Device turn ons and enters a WAIT state
 * 2. user presses Charge Button (i.e. code should be waiting for signal on GPIO)
 * 4. Check for battery voltage
 *      If 0 < voltage < 1.4V & battery is present
 *      then
 *        check battery temperatur
 *        if temp <  (need temp value)
 *          continously checking temp
 *          while voltage is < 1.4 OR N0. Cycles < 4
 *            charging 30 minutes
 *            rest 30 seconds
 *          done
 *          eject andf display final temp, time taken to charge,
 */

#include <Wire.h>

#define CHARGE_BUTTON 13  //GPIO for charge start button Pin A12
#define VOLTAGE_SENSOR 12 //pin for checking battery voltage
#define CHARGE_CONTROL 33 //control charging ON and OFF states

#define V_MIN 0.6 //0.6 is the minimum acceptable voltage to start charging
#define V_MAX 1.4 //1.4 max voltage
#define TEMP_MAX 30 //max temp = 30 C
#define CYCLE_MAX 4 //max number of cycles before battery ejects
#define CHAREG_TIME 1800000 //30 minutes = 1800000 milliseconds
#define REST_TIME 300000 //30 seconds = 30000 milliseconds
#define TEMP_SENSOR_BASE_ADDRESS 0x5A // base address for MLX90614 is 0x5A

float voltage = 0.0; //store battery voltage
float temp = 0.0; //battery temp
unsigned long chargeStart = 0;
unsigned long chargeTotal = 0;



//states
enum States{WAIT, CHECK_VOLT, CHARGE, DONE};
State currentState = WAIT; //when user powers on the device, the device enters into the WAIT state


void setup() {
  // put your setup code here, to run once: 
  //
  Serial.begin(115200); //setup baud rate = 115200
  Wire.begin(); //setup/initialize I2C comms on SDA/SCL

  pinMode(CHARGE_BUTTON, INPUT_PULL); //

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
