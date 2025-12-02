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

double kP = 1;
double kI = 0;
double kD = 0;

double target = 100; //REPLACE 100 WITH MIDPOINT
double totalError, previousError, changeError, PIDOut = 0;

/**
 * Arduino setup function
 */
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== STM32 VL53L4CD Test ===");

  // Initialize GPIO pins for TMC2209
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Set initial states
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);     // Clockwise
  digitalWrite(ENABLE_PIN, LOW);   // ENABLE the driver first (active low)

  delay(100);  // Give driver time to wake up

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C

  Serial.println("Initializing VL53L4CD...");

  // Initialize sensor
  sensor.VL53L4CD_Off();
  Serial.println("off");

  sensor.begin();
  Serial.println("begin");

  sensor.InitSensor();

  // Initialize the sensor
  // if (sensor.VL53L4CD_SensorInit() != VL53L4CD_ERROR_NONE) {
  //   Serial.println("ERROR: Sensor init failed!");
  //   while (1) {
  //     delay(100);
  //   }
  // }

  Serial.println("Sensor initialized successfully");

  // Set timing budget and inter-measurement period
  // Timing budget: 20-200ms (time for one measurement)
  // Inter-measurement: 0ms for continuous mode
  sensor.VL53L4CD_SetRangeTiming(200, 0);  // 50ms timing, continuous
  Serial.println("Set Timing");

  sensor.VL53L4CD_StartRanging();

  // // Start ranging
  // if (sensor.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
  //   Serial.println("ERROR: Failed to start ranging!");
  //   while (1) {
  //     delay(100);
  //   }
  // }

  Serial.println("Ranging started");
  Serial.println("System ready!\n");

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
    Serial.print("Distance: ");
    Serial.print(distance_mm);
    Serial.print(" mm, Status: ");
    Serial.print(range_status);
    Serial.print(", Signal: ");
    Serial.print(results.signal_per_spad_kcps / 2048.0);
    Serial.print(" kcps, Ambient: ");
    Serial.print(results.ambient_per_spad_kcps / 2048.0);
    Serial.println(" kcps");

    // // Control stepper motor based on distance
    // // Only respond to valid measurements (status 0)
    // if (range_status == 0) {
    //   // Set direction based on distance
    //   if (distance_mm < 150) {
    //     digitalWrite(DIR_PIN, LOW);  // Counter-clockwise
    //   } else {
    //     digitalWrite(DIR_PIN, HIGH); // Clockwise
    //   }
    // }
  }
  

  double e = target - distance_mm;
  totalError += e;
  changeError = e - previousError;
  PIDOut = (kP * e) + (kI * totalError) + (kD * changeError);
  
  if (e > 0) {
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(STEP_PIN, HIGH);
    delay(1/abs(PIDOut));
    digitalWrite(STEP_PIN, LOW);
  }else if (e < 0){
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(STEP_PIN, HIGH);
    delay(1/abs(PIDOut));
    digitalWrite(STEP_PIN, LOW);
  }
  previousError = e;

  // // Generate continuous steps based on last valid distance reading
  // // This runs independently of sensor data updates for smooth motion
  // if (distance_mm < 100) {
  //   // Close - slow speed (50 steps/sec = 20ms per step)
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(5000);  // 5ms pulse width (very conservative)
  //   digitalWrite(STEP_PIN, LOW);
  //   delay(15);  // 15ms delay = ~50 steps/sec
  // } else if (distance_mm < 300) {
  //   // Medium distance - medium speed (100 steps/sec = 10ms per step)
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(5000);  // 5ms pulse width
  //   digitalWrite(STEP_PIN, LOW);
  //   delay(5);  // 5ms delay = ~100 steps/sec
  // } else if (distance_mm < 500) {
  //   // Far - fast speed (200 steps/sec = 5ms per step)
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(3000);  // 3ms pulse width
  //   digitalWrite(STEP_PIN, LOW);
  //   delay(2);  // 2ms delay = ~200 steps/sec
  // } else {
  //   // Very far or no object - no steps
  //   delay(10);
  // }
}
