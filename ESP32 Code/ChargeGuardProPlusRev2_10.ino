#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_SSD1306.h>  // Library for SSD1306 OLED
#include <Arduino.h>

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

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Global variables
float voltage = 0.0;
float temp = 0.0;
unsigned long chargeStart = 0;
unsigned long chargeTotal = 0;
unsigned long lastPollTime = 0;
const unsigned long POLL_INTERVAL = 30000;

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

  // ADC Configuration for Accurate Readings
  analogSetWidth(12); // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db); // Set attenuation to measure up to 3.3V

  // OLED setup
  if (!OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Default I2C address for SSD1306
    Serial.println("OLED Setup Failed");
    for (;;);
  }

  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
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
        OLED.println("State: WAIT");
        OLED.println("Press START");
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

    case CHECK_BATTERY:
      Serial.println("In CHECK_BATTERY state");
      clearDisplay();
      if (isBatteryPresent()) {
        OLED.println("Battery Detected");
        OLED.display();
        delay(2000);
        voltage = readBatteryVoltage();
        clearDisplay();
        OLED.print("Voltage: ");
        OLED.print(voltage, 2);
        OLED.println("V");
        OLED.display();
        delay(2000);

        if (checkVoltage()) {
          currentState = CHARGE;
        } else {
          currentState = WAIT;
        }
      } else {
        clearDisplay();
        Serial.print("No Battery\n");
        OLED.println("No Battery!");
        OLED.println("Returning...");
        OLED.display();
        delay(2000);
        currentState = WAIT;
      }
      break;

    case CHARGE:
      clearDisplay();
      OLED.println("CHARGING");
      OLED.display();
      delay(2000);
      chargeTotal = 0; // Reset charge time at the beginning of each charge
      chargeStart = millis();

      for (int i = 0; i <= CYCLE_MAX; i++) {
        while (millis() - chargeStart < CHARGE_TIME) {
          temp = temp_sensor.readObjectTempC();

          if (temp != lastTemp || voltage != lastVoltage) {
            clearDisplay();
            OLED.print("Temp: ");
            OLED.print(temp, 1);
            OLED.println(" C");
            OLED.print("Volt: ");
            OLED.print(voltage, 2);
            OLED.println(" V");
            OLED.display();
            lastTemp = temp;
            lastVoltage = voltage;
          }

          delay(500);

          if (temp <= TEMP_MIN || temp >= TEMP_MAX) {
            Serial.println("ERROR: Temp out of range.");
            digitalWrite(CHARGE_CONTROL, LOW);
            clearDisplay();
            OLED.println("Temp Error!");
            OLED.display();
            delay(2000);
            currentState = DONE;
            return;
          }

          if (millis() - lastPollTime >= POLL_INTERVAL) {
            lastPollTime = millis();
            voltage = readBatteryVoltage();

            if (!checkVoltage()) {
              currentState = DONE;
              return;
            }
          }

          chargeTotal = millis() - chargeStart;
        }
      }

      currentState = DONE;
      break;

    case DONE:
      Serial.println("In DONE state");
      displayChargingStats(temp, chargeTotal, voltage);
      digitalWrite(SOLENOID_PIN, HIGH);
      delay(500);
      digitalWrite(SOLENOID_PIN, LOW);
      currentState = WAIT;
      break;
  }
}

void clearDisplay() {
  OLED.clearDisplay();
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
    clearDisplay();
    OLED.println("Battery Fully Charged");
    OLED.display();
    delay(5000);
    //currentState = DONE;
    return 0;      
  } else if (voltage < V_MIN) {
    clearDisplay();
    OLED.println("ERROR - Battery Below Minimum Charging Voltage - Discard Battery");
    OLED.display();
    delay(5000);
    //return 0;        
  } else {
    clearDisplay();
    OLED.println("Unknown Error: Returning to WAIT State");
    OLED.display();
    delay(5000);
    //currentState = DONE;
    return 0;  
  }
  return 0;  
}

void displayChargingStats(float temp, unsigned long chargeTotal, float voltage) {
  clearDisplay();
  OLED.println("Charging Done");
  OLED.print("Temp: ");
  OLED.print(temp, 1);
  OLED.println("C");
  OLED.print("Time: ");
  OLED.print(chargeTotal / 60000);
  OLED.println(" min");
  OLED.print("Volt: ");
  OLED.print(voltage, 2);
  OLED.println(" V");
  OLED.display();
  delay(5000);
  clearDisplay();
}
