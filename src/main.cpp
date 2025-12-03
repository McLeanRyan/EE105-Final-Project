/**
 * STM32L432KC - Stepper Motor Control with TMC2209 and VL53L4CD Distance Sensor
 * Using Arduino Framework
 *
 * Hardware:
 * - Nucleo-L432KC (STM32L432KC microcontroller)
 * - Adafruit TMC2209 Stepper Driver (STEP/DIR interface)
 * - VL53L4CD ToF Distance Sensor (I2C interface)
 *
 * Pin Connections:
 * - PA0: STEP (TMC2209)
 * - PA1: DIR (TMC2209)
 * - PA4: ENABLE (TMC2209, active low)
 * - PB6: I2C1_SCL (VL53L4CD)
 * - PB7: I2C1_SDA (VL53L4CD)
 * - PA2: USART2_TX (debug via ST-LINK)
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <TMC2209.h>
#include <SoftwareSerial.h>

// Pin definitions
#define STEP_PIN        PA0
#define DIR_PIN         PA1
#define ENABLE_PIN      PA4

// VL53L4CD sensor object
VL53L4CD sensor(&Wire, A1);  // A1 is a dummy pin, sensor uses default I2C address

// Global variables
uint16_t distance_mm = 0;
uint8_t range_status = 0;
VL53L4CD_Result_t results;

TMC2209 stepper;
SoftwareSerial TMCSerial(PA10, PA9);  // RX, TX

double kP = 0.0015;   // 50% of original
double kI = 0.0000001;   // 25% of original  
double kD = 0;   // 50% of original

double target = 90; //REPLACE 100 WITH MIDPOINT
double totalError, previousError, changeError, PIDOut = 0;

unsigned int lastTime = 0;

/**
 * Arduino setup function
 */
void setup() {
  // Initialize serial for debugging
  // Serial.begin(115200);
  // delay(1000);
  // Serial.println("\n\n=== STM32 VL53L4CD Test ===");
  
  TMCSerial.begin(115200);
  delay(1000);
  stepper.setup(TMCSerial);
  stepper.setRunCurrent(100);
  // stepper.enableAutomaticCurrentScaling();
  // stepper.enableAutomaticGradientAdaptation();
  stepper.enableCoolStep();
  stepper.enable();

  // Initialize GPIO pins for TMC2209
  // pinMode(STEP_PIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
  // pinMode(ENABLE_PIN, OUTPUT);

  // // Set initial states
  // digitalWrite(STEP_PIN, LOW);
  // digitalWrite(DIR_PIN, HIGH);     // Clockwise
  // digitalWrite(ENABLE_PIN, LOW);   // ENABLE the driver first (active low)

  delay(100);  // Give driver time to wake up

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C

  // Serial.println("Initializing VL53L4CD...");

  // Initialize sensor
  sensor.VL53L4CD_Off();
  // Serial.println("off");

  sensor.begin();
  // Serial.println("begin");

  sensor.InitSensor();

  // Initialize the sensor
  // if (sensor.VL53L4CD_SensorInit() != VL53L4CD_ERROR_NONE) {
  //   Serial.println("ERROR: Sensor init failed!");
  //   while (1) {
  //     delay(100);
  //   }
  // }

  // Serial.println("Sensor initialized successfully");

  // Set timing budget and inter-measurement period
  // Timing budget: 20-200ms (time for one measurement)
  // Inter-measurement: 0ms for continuous mode
  sensor.VL53L4CD_SetRangeTiming(20, 0); 
  // Serial.println("Set Timing");

  sensor.VL53L4CD_StartRanging();

  // // Start ranging
  // if (sensor.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
  //   Serial.println("ERROR: Failed to start ranging!");
  //   while (1) {
  //     delay(100);
  //   }
  // }

  // Serial.println("Ranging started");
  // Serial.println("System ready!\n");

  // Enable stepper driver
  digitalWrite(ENABLE_PIN, LOW);  // Enable (active low)

  
}

/**
 * Arduino main loop
 */
void loop() {
  uint8_t data_ready = 0;

  // Check if new data is available
  sensor.VL53L4CD_CheckForDataReady(&data_ready);

  if (data_ready) {
    // Get measurement results
    sensor.VL53L4CD_GetResult(&results);

    // Clear interrupt to allow next measurement
    sensor.VL53L4CD_ClearInterrupt();

    // Extract distance and status
    distance_mm = results.distance_mm;
    range_status = results.range_status;

    // Print results
    // Serial.print("Distance: ");
    // Serial.print(distance_mm);
    // Serial.print(" mm, Status: ");
    // Serial.print(range_status);
    // Serial.print(", Signal: ");
    // Serial.print(results.signal_per_spad_kcps / 2048.0);
    // Serial.print(" kcps, Ambient: ");
    // Serial.print(results.ambient_per_spad_kcps / 2048.0);
    // Serial.println(" kcps");

    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;  // Convert to seconds
    
    if (dt <= 0) return;  // Guard against zero/negative dt

    double e = target - distance_mm;
    totalError += (e * dt);
    changeError = (e - previousError) / dt;
    PIDOut = (kP * e) + (kI * totalError) + (kD * changeError);
    
    if (PIDOut > 0) {
      stepper.disableInverseMotorDirection();
      stepper.moveAtVelocity(20000*abs(PIDOut));
    }else if (PIDOut < 0){
      stepper.enableInverseMotorDirection();
      stepper.moveAtVelocity(20000*abs(PIDOut));
    } else {
      stepper.moveAtVelocity(0);
    }

    previousError = e;
    lastTime = now;
  }
}
