#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_SH110X.h>  // Library for OLED

// Pin definitions
#define CHARGE_BUTTON 27
#define VOLTAGE_SENSOR A2
#define CHARGE_CONTROL 12
#define SOLENOID_PIN 26

// Voltage and temperature thresholds
#define V_MIN 0.6
#define V_MAX 5
#define TEMP_MIN 10
#define TEMP_MAX 30
#define CYCLE_MAX 4
#define CHARGE_TIME 1800000
#define REST_TIME 300000

// Temperature sensor base address
#define TEMP_SENSOR_BASE_ADDRESS 0x5A

// Sensor setup
Adafruit_MLX90614 temp_sensor = Adafruit_MLX90614();
Adafruit_SH1107 OLED(128, 128, &Wire, -1);

// Global variables
float voltage = 0.0;
float temp = 0.0;
unsigned long chargeStart = 0;
unsigned long chargeTotal = 0;
unsigned long lastPollTime = 0;
const unsigned long POLL_INTERVAL = 60000;

// State definitions
enum State_type { WAIT, CHECK_BATTERY, CHARGE, DONE };
State_type currentState = WAIT;

// Interrupt flag
volatile bool buttonPressed = false;

// Interrupt Service Routine (ISR)
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true; // Set the flag when the button is pressed
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  Wire.begin();
  temp_sensor.begin();

  // OLED setup
  if (!OLED.begin(0x3C, true)) {
    Serial.println("OLED Setup Failed");
    for (;;);
  }

  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(SH110X_WHITE);
  OLED.display();

  // Pin setup
  pinMode(CHARGE_BUTTON, INPUT_PULLUP); // Use pull-up resistor for button
  pinMode(CHARGE_CONTROL, OUTPUT);
  digitalWrite(CHARGE_CONTROL, LOW);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  // Attach interrupt for button
  attachInterrupt(digitalPinToInterrupt(CHARGE_BUTTON), handleButtonPress, FALLING);

  Serial.println("Setup completed");
}

void loop() {
  float lastTemp = -1.0;  // Last displayed temperature
  float lastVoltage = -1.0; // Last displayed voltage

  switch (currentState) {
    case WAIT:
      Serial.println("In WAIT state");
      
      static bool displayUpdated = false;
      if (!displayUpdated) {
        clearDisplay();
        OLED.println("Current State: WAIT\nPress START Button to\nBegin Charging");
        OLED.display();
        displayUpdated = true;
      }

      // Check interrupt flag
      if (buttonPressed) {
        buttonPressed = false; // Clear the flag
        delay(250); // Debounce delay
        Serial.println("Button pressed. Transitioning to CHECK_BATTERY state.");
        currentState = CHECK_BATTERY;
        displayUpdated = false; // Reset for the next state
      }
      break;
     
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
            temp = temp_sensor.readObjectTempC();
            digitalWrite(CHARGE_CONTROL, HIGH);
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
                clearDisplay();
                OLED.println("ERROR - Temp Exceeded Range!");
                OLED.display();
                delay(2000);
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
      delay(500); // Wait before resetting
      Serial.print("\nEjecting using Solenoid\n");
      digitalWrite(SOLENOID_PIN, HIGH); //eject battery using solenoid
      delay(500);
      Serial.print("\nRetracting Solenoid\n");
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
