/*
 * ECE411 Practicum Project - FallTerm 2024 Portland State University
 * Group 2
 * Cody Reid, Cuauhtemoc Gomez Angeles, Daniel Anishchenko, Tyler Tran
 * ChargeGuardPro+ Battery Charging Solution
 */

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_SSD1306.h> //library for OLED

#define CHARGE_BUTTON 12  //GPIO for charge start button Pin A12
#define VOLTAGE_SENSOR 33 //pin for checking battery voltage
#define CHARGE_CONTROL 27 //control charging ON and OFF states
#define SOLENOID_PIN 26 //GPIO pin for controlling solenoid using a MOSFET

#define V_MIN 0.6 //0.6 is the minimum acceptable voltage to start charging
#define V_MAX 1.4 //1.4 max voltage
#define TEMP_MIN 10 // max temp = 10 C
#define TEMP_MAX 30 //max temp = 30 C
#define CYCLE_MAX 4 //max number of cycles before battery ejects
#define CHARGE_TIME 1800000 //30 minutes = 1800000 milliseconds
#define REST_TIME 300000 //30 seconds = 30000 milliseconds
#define TEMP_SENSOR_BASE_ADDRESS 0x5A // base address for MLX90614 is 0x5A

// Temperature sensor setup
Adafruit_MLX90614 temp_sensor = Adafruit_MLX90614();

// OLED Device setup
Adafruit_SSD1306 OLED = Adafruit_SSD1306();

float voltage = 0.0; //store battery voltage
float temp = 0.0; //battery temp
unsigned long chargeStart = 0.0;
unsigned long chargeTotal = 0.0;

// Function Declaration
int checkVoltage();
int isBatteryPresent();
float readBatteryVoltage();
void clearDisplay();
void displayChargingStats(float temp, unsigned long chargeTotal);

// States
enum State_type { WAIT, CHECK_BATTERY, CHARGE, DONE };
State_type currentState = WAIT; //when user powers on the device, the device enters into the WAIT state

void setup() {
  Serial.begin(115200); //setup baud rate = 115200
  Wire.begin(); //setup/initialize I2C comms on SDA/SCL

  temp_sensor.begin();  //setup MLX90614 Temperature sensor

  // Setup OLED display
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C); //OLED base address on ESP32 -> 0x3C
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  
  pinMode(CHARGE_BUTTON, INPUT); //initialize CHARGE_BUTTON pin as an input
  pinMode(CHARGE_CONTROL, OUTPUT); 
  digitalWrite(CHARGE_CONTROL, LOW); //setting the CHARGE_CONTROL with a low, charging is in an OFF state initially
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
}

void loop() {
  switch (currentState) {
    case WAIT:
      clearDisplay();
      OLED.println("Current State: WAIT - Press START Button to Begin Charging\n");
      OLED.display();
      
      // If Start button has been pressed, transition to CHECK_BATTERY state
      if (digitalRead(CHARGE_BUTTON) == LOW) {
        currentState = CHECK_BATTERY;
      }
      break;
    
    case CHECK_BATTERY:
      clearDisplay();
      
      voltage = readBatteryVoltage();  
      
      if (isBatteryPresent()) {
        OLED.println("Battery Detected!!!");
        OLED.display();
        if(checkVoltage()){
          currentState = CHARGE;
        } else {
          currentState = WAIT;
        }
      } else {
        OLED.println("ERROR - Battery Not Present!!!");
        OLED.display();
        currentState = WAIT;
      }

      break;
    
    case CHARGE:
      digitalWrite(CHARGE_CONTROL, HIGH); //Start charging
      chargeTotal = 0; //reset charge time at the beginning of each charge
      
      for(int i = 0; i <= CYCLE_MAX; i++) {
        // Use millis() to measure charging time without blocking the program
        unsigned long cycleStartTime = millis();
        while(millis() - chargeStart < CHARGE_TIME) {
          temp = temp_sensor.readObjectTempC();
          clearDisplay();
          OLED.print("Current Temp: ");
          OLED.print(temp);
          OLED.print(" C");

          if(temp <= TEMP_MIN || temp >= TEMP_MAX) {
            // Stop charging if temp is out of range
            digitalWrite(CHARGE_CONTROL, LOW);
            OLED.println("ERROR - Temperature Exceeded Range Boundaries During Charging Process");
            OLED.display();
            currentState = WAIT; //exit charge state
            return;
          }
          if(!checkVoltage()) {
            digitalWrite(CHARGE_CONTROL, LOW); //voltage is out of range, stop charging
            OLED.println("ERROR - Voltage Exceeded Limits During Charging Process");
            OLED.display();
            currentState = WAIT;
            return;
          }
          if((voltage = readBatteryVoltage()) >= V_MAX) {
            digitalWrite(CHARGE_CONTROL, LOW); //voltage is at max, stop charging
            OLED.println("Battery Fully Charged - Charging Done");
            OLED.display();
            currentState = DONE;
            return;
          }
          chargeTotal += millis() - chargeStart; //sum up the total time
          cycleStartTime = millis(); //reset chargeStart for next while loop interval
        }
      }
      break;  
    
    case DONE:
      displayChargingStats(temp, chargeTotal);
      delay(5000); // Wait before resetting
    digitalWrite(SOLENOID_PIN, HIGH); //eject battery using solenoid
      currentState = WAIT;
      break;
  }
}

void clearDisplay() {
  OLED.clearDisplay();
  OLED.setCursor(0, 0);
}

float readBatteryVoltage() {
  float ADC_voltage = analogRead(VOLTAGE_SENSOR);
  // The analog digital value read by the ESP32 ranges from 0-4095
  return (ADC_voltage / 4095) * 3.3; // may need to change to 5V
}

int isBatteryPresent() {
  if(readBatteryVoltage() > 0.1) {
    return 1;
  } else {
    return 0;
  }
}

int checkVoltage() {
  voltage = readBatteryVoltage();
  if (voltage > V_MIN && voltage <= V_MAX) {
    return 1;
  } else if (voltage >= V_MAX) {
    OLED.println("Battery Fully Charged");
    OLED.display();
    return 0;      
  } else if (voltage < V_MIN) {
    OLED.println("ERROR - Battery Below Minimum Charging Voltage - Discard Battery");
    OLED.display();
    return 0;        
  } else {
    OLED.println("Unknown Error: Returning to WAIT State");
    OLED.display();
    return 0;  
  }  
}

void displayChargingStats(float temp, unsigned long chargeTotal) {
  clearDisplay();
  OLED.println("Charging Complete");
  OLED.print("Final Temp: ");
  OLED.print(temp);
  OLED.println(" C");
  OLED.print("Total Charge Time: ");
  OLED.print(chargeTotal / 60000); // Time in minutes
  OLED.println(" min");
  OLED.display();
}
