/*
 * ECE411 Practicum Project - FallTerm 2024 Portland State University
 * Group 2
 * Cody Reid, Cuauhtemoc Gomez Angeles, Daniel Anishchenko, Tyler Tran
 * ChargeGuardPro+ Battery Charging Solution
 */

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_SH110X.h>  //library for OLED

#define CHARGE_BUTTON 27  //GPIO for charge start button Pin 
#define VOLTAGE_SENSOR 33 //pin for checking battery voltage --> Board Call Out 
#define CHARGE_CONTROL 12 //control charging ON and OFF states
#define SOLENOID_PIN 26 //GPIO pin for controlling solenoid using a MOSFET

#define V_MIN 0.6 //0.6 is the minimum acceptable voltage to start charging
#define V_MAX 1.6 //Changed to 2 for troubleshooting, needs to be 1.4 in final rev
#define TEMP_MIN 10 // max temp = 10 C
#define TEMP_MAX 30 //max temp = 30 C
#define CYCLE_MAX 4 //max number of cycles before battery ejects
#define CHARGE_TIME 1800000 //30 minutes = 1800000 milliseconds
#define REST_TIME 300000 //30 seconds = 30000 milliseconds
#define TEMP_SENSOR_BASE_ADDRESS 0x5A // base address for MLX90614 is 0x5A

// Temperature sensor setup
Adafruit_MLX90614 temp_sensor = Adafruit_MLX90614();


// OLED Device setup
Adafruit_SH1107 OLED(128, 128, &Wire, -1);

float voltage = 0.0; //initialize battery voltage
float temp = 0.0; //initialize battery temp
unsigned long chargeStart = 0; //initialize charge start time
unsigned long chargeTotal = 0; //initialize total charge time
unsigned long lastPollTime = 0; // to track 60-second intervals
const unsigned long POLL_INTERVAL = 60000; // 60 seconds

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
  Serial.print("At the top of setup stage\n");
  Wire.begin(); //setup/initialize I2C comms on SDA/SCL

  temp_sensor.begin();  //setup MLX90614 Temperature sensor

  if(!OLED.begin(0x3C, true)){
    Serial.print("OLED Setup Failed\n");
    for(;;);//STOP!!! Forever FOR loop
  }
  
  // Setup OLED display
  OLED.begin(0x3C, true); //OLED base address on ESP32 -> 0x3C
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(SH110X_WHITE);
  OLED.display();
  
  pinMode(CHARGE_BUTTON, INPUT); //initialize CHARGE_BUTTON pin as an input
  pinMode(CHARGE_CONTROL, OUTPUT); 
  digitalWrite(CHARGE_CONTROL, LOW); //charging is in an OFF state initially
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  Serial.print("At the bottom of setup stage\n");
}

void loop() {
  Serial.print("At the top of loop stage\n");
  delay(5000);
  switch (currentState) {
    case WAIT:
      Serial.print("In WAIT state\n");
      //delay(10000);
      clearDisplay();
      OLED.println("Current State: WAIT\nPress START Button to\nBegin Charging\n");
      OLED.display();

      // If Start button has been pressed, transition to CHECK_BATTERY state
      Serial.print("Status of CHARGE_BUTTON:  "); Serial.print(digitalRead(CHARGE_BUTTON)); Serial.print("\n ");
      if(digitalRead(CHARGE_BUTTON) == HIGH) {
        delay(10000);
        //Serial.print("Line 87\n");
        currentState = CHECK_BATTERY;
        break;
      }
     
      //Serial.print("line 92\n");
      break;
    case CHECK_BATTERY:
      //delay(10000);
      Serial.print("At the top of CHECK_BATTERY\n"); 
      clearDisplay();
      //voltage = readBatteryVoltage();
      Serial.print("Battery voltage in CHECK_BATTERY state: ");
      //Serial.println(voltage);
      
      if (isBatteryPresent()) {
        Serial.print("Battery Dtected!!!\n");
        clearDisplay();
        OLED.println("Battery Detected!!!");
        OLED.display();
        delay(5000);
        voltage = readBatteryVoltage();
        OLED.print("Current Voltage Value\nof Battery: ");
        OLED.println(voltage);
        OLED.display();
        delay(5000);

        //clearDisplay();
        //the below line of code should be if(checkVoltage()){ putting a 1 to troubleshoot code
        if(1){
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
      OLED.println("Entering CHARGE state\nStating Charging NOW");
      delay(500);
      digitalWrite(CHARGE_CONTROL, HIGH); //Start charging
      delay(500);
      chargeTotal = 0; //reset charge time at the beginning of each charge
      chargeStart = millis();
      for(int i = 0; i <= CYCLE_MAX; i++) {
        unsigned long cycleStartTime = millis();
        
        while(millis() - chargeStart < CHARGE_TIME) {
          temp = 25.5; //using constant for tesing
          //temp = temp_sensor.readObjectTempC();
          //Serial.print("Current Temp: ");
          //Serial.println(temp);
          //delay(5000);
          //clearDisplay();
          //delay(2000);
          OLED.print("Current Temp: ");
          OLED.print(temp);
          OLED.print(" C\n");
          OLED.display();
          delay(2000);
          clearDisplay();
        
          

          if(temp <= TEMP_MIN || temp >= TEMP_MAX) {
            Serial.print("in the if(temp <= TEMP_MIN || temp >= TEMP_MAX block\n");
            delay(500);
            digitalWrite(CHARGE_CONTROL, LOW);
            delay(500);
            OLED.println("ERROR - Temperature Exceeded Range Boundaries During Charging Process");
            OLED.display();
            currentState = DONE; //exit charge state
            return;
          }
          //replaceing below line with if(0) for troubleshooting 
          //should be if(!checkVoltage) {
          //Serial.print("Value of checkVoltage Func: "); Serial.print(digitalRead(!checkVoltage()));Serial.print("\n");
          /*if(!checkVoltage()) {
            Serial.print("In the if(!checkVoltage) block\n");
            delay(500);
            digitalWrite(CHARGE_CONTROL, LOW); //voltage out of range, stop charging
            delay(500);
            OLED.println("ERROR - Voltage Exceeded Limits During Charging Process");
            OLED.display();

            currentState = WAIT;
            return;
          }*/
          // Poll voltage every 60 seconds
          if (millis() - lastPollTime >= POLL_INTERVAL) {
            lastPollTime = millis();

            // Stop charging to measure voltage
            delay(500);
            digitalWrite(CHARGE_CONTROL, LOW); // Stop charging
            delay(500); 
      
            //voltage = readBatteryVoltage(); // Measure voltage
            if(!checkVoltage()){
              OLED.print("Voltage Exceeded Bounds\n");
              OLED.display();
              delay(2000);
              currentState = DONE;

            }

            //clearDisplay();
            OLED.print("Voltage Last Poll: ");
            OLED.println(readBatteryVoltage());
            OLED.display();
            delay(2000);
            clearDisplay();
        
            // Restart charging
            delay(200);
            digitalWrite(CHARGE_CONTROL, HIGH);
            delay(200);
          }

          //voltage = readBatteryVoltage();

          /*if(voltage >= V_MAX) {
            Serial.print("In the if(voltage >= V_MAX) block\n");
            //delay(500);
            digitalWrite(CHARGE_CONTROL, LOW); //voltage is at max, stop charging
            delay(200);
            OLED.println("Battery Fully Charged - Charging Done");
            OLED.display();
            currentState = DONE;
            return;
          }*/

          chargeTotal += millis() - chargeStart; //sum up the total time
          cycleStartTime = millis(); //reset chargeStart for next while loop interval
        }
        currentState = DONE;
      }
      break;  
    
    case DONE:
      Serial.print("In the DONE state\n");
      displayChargingStats(temp, chargeTotal);
      delay(5000); // Wait before resetting
      digitalWrite(SOLENOID_PIN, HIGH); //eject battery using solenoid
      delay(5000);
      digitalWrite(SOLENOID_PIN, LOW);
  
      currentState = WAIT;
      break;
  }
}

void clearDisplay() {
  OLED.clearDisplay();
  OLED.setCursor(0, 0);
  OLED.display();
  OLED.setCursor(0, 0);
}

int isBatteryPresent() {
  if(readBatteryVoltage() > 0.1) {
    Serial.print("In isBatteryPresent func\n");
    return 1;
  } else {
    return 0;
  }
}

float readBatteryVoltage() {
  float ADC_voltage = analogRead(VOLTAGE_SENSOR);
  //Serial.print("Line 194\n");
  //delay(10000);
  if (ADC_voltage > 0) {
    //Serial.print("In the readBatteryVoltage func, returning voltage value: "); Serial.print((ADC_voltage / 4095.0) * 3.3); Serial.print("\n");
    return ((ADC_voltage / 4095.0) * 3.3)*2;
  } else {
    Serial.println("Error: ADC_voltage is zero. Returning 0.");
    return 0.0;
  }
}

int checkVoltage() {
  voltage = readBatteryVoltage();
  
  Serial.print("Voltage in checkVoltage(): ");
  Serial.println(voltage);

  if (voltage > V_MIN && voltage <= V_MAX) {
    return 1; // Voltage within acceptable range
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
  delay(10000);
  clearDisplay();
}
