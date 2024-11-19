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
#define VOLTAGE_SENSOR A2 //pin for checking battery voltage --> Board Call Out 
#define CHARGE_CONTROL 12 //control charging ON and OFF states
#define SOLENOID_PIN 26 //GPIO pin for controlling solenoid using a MOSFET

#define V_MIN 0.6 //0.6 is the minimum acceptable voltage to start charging
#define V_MAX 5 //Changed to 2 for troubleshooting, needs to be 1.4 in final rev
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
  float lastTemp = -1.0;  // To track last displayed temperature
  float lastVoltage = -1.0; // To track last displayed voltage

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
    OLED.println("Entering CHARGE state\nStarting Charging NOW");
    OLED.display();
    delay(2500);
    clearDisplay();
    chargeTotal = 0; // Reset charge time at the beginning of each charge
    chargeStart = millis();

    for (int i = 0; i <= CYCLE_MAX; i++) {
        unsigned long cycleStartTime = millis();

        while (millis() - chargeStart < CHARGE_TIME) {
            // Read current temperature
            temp = 25.5; // Replace with actual temp sensor reading if needed

            // Update OLED display only if temperature or voltage values have changed
            if (temp != lastTemp || voltage != lastVoltage) {
                if (temp != lastTemp) {
                    // Clear the temperature display area and update
                    OLED.fillRect(0, 16, 128, 16, 0); // Clear a rectangle for temperature display
                    OLED.setCursor(0, 16);
                    OLED.print("Temp: ");
                    OLED.print(temp, 1); // Display temperature with 1 decimal place
                    OLED.println(" C");
                }

                if (voltage != lastVoltage) {
                    // Clear the voltage display area and update
                    OLED.fillRect(0, 32, 128, 16, 0); // Clear a rectangle for voltage display
                    OLED.setCursor(0, 32);
                    OLED.print("Voltage: ");
                    OLED.print(voltage, 2); // Display voltage with 2 decimal places
                    OLED.println(" V");
                }

                OLED.display();

                // Update last displayed values
                lastTemp = temp;
                lastVoltage = voltage;
            }

            delay(500); // Brief delay to avoid excessive updates

            // Check if temperature is out of range
            if (temp <= TEMP_MIN || temp >= TEMP_MAX) {
                Serial.println("ERROR: Temperature out of range during charging.");
                digitalWrite(CHARGE_CONTROL, LOW); // Stop charging
                OLED.println("ERROR - Temp Exceeded Range!");
                OLED.display();
                currentState = DONE;
                return;
            }

            // Poll voltage only during the polling interval
            if (millis() - lastPollTime >= POLL_INTERVAL) {
                lastPollTime = millis();

                // Stop charging temporarily to measure voltage
                digitalWrite(CHARGE_CONTROL, LOW); // Stop charging
                delay(500);

                // Read voltage
                voltage = readBatteryVoltage();

                // Check voltage only during polling
                if (!checkVoltage()) {
                    Serial.println("ERROR: Voltage out of range during charging.");
                    OLED.println("ERROR - Voltage Out of Range!");
                    OLED.display();
                    currentState = DONE;
                    return;
                }
              
                // Restart charging after voltage check
                digitalWrite(CHARGE_CONTROL, HIGH);
            }

            // Update total charge time
            chargeTotal = millis() - chargeStart;
        }
    }

    currentState = DONE; // Exit after charging cycles are complete
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
    delay(5000);
    currentState = DONE;
    //return 0;      
  } else if (voltage < V_MIN) {
    OLED.println("ERROR - Battery Below Minimum Charging Voltage - Discard Battery");
    OLED.display();
    delay(5000);
    //return 0;        
  } else {
    OLED.println("Unknown Error: Returning to WAIT State");
    OLED.display();
    delay(5000);
    currentState = DONE;
    //return 0;  
  }
  return 0;  
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
