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
#include <HardwareTimer.h>

// Pin definitions
#define STEP_PIN        PA0
#define DIR_PIN         PA1
#define ENABLE_PIN      PA4

// VL53L4CD sensor object
VL53L4CD sensor(&Wire, A1);  // A1 is a dummy pin, sensor uses default I2C address

// Global variables
uint16_t distance_mm = 0;
uint8_t range_status = 0;
// uint16_t distance_mm2 = 0;
// uint8_t range_status2 = 0;
VL53L4CD_Result_t results;

TMC2209 stepper;
SoftwareSerial TMCSerial(PA10, PA9);  // RX, TX


//-------------------------------------------- PID control variables ---
double kP = 0.039605;
double kI = 0.0009713; 
double kD = 0.40373;

// double kP = 0.13043;
// double kI = 0.0057442; Faster response from OG 
// double kD = 0.74039;

//-----------------------------------------------------------------------

double target = 125; //REPLACE 100 WITH MIDPOINT
double minDist = 50;
double maxDist = 150;
double totalError, previousError, changeError, PIDOut = 0;

unsigned int lastTime = 0;

// --- Position control variables ---
volatile long currentPositionSteps = 0;   // step counter maintained by MCU
volatile long targetPositionSteps = 0;    // desired step index
volatile bool stepState = false;          // for half-period toggling of STEP pin
unsigned long lastStepTimeUs = 0;         // timestamp for step state machine
// Software-based step generator will be used for stepping

// Hardware / machine-specific constants (edit to match your mechanics)
const double STEPS_PER_REV = 200.0;       // full steps per motor revolution
const double MICROSTEPS = 16.0;           // microsteps set on the driver (e.g. 16)
const double LEAD_SCREW_MM_PER_REV = 2.0; // mm travel per revolution (example)
const double STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_MM_PER_REV;

// Stepper pulse timing limits (in Hz) - lowered for safe initial testing
const unsigned long STEP_FREQ_MIN = 20;      // min stepping frequency (Hz)
const unsigned long STEP_FREQ_MAX = 2000;    // max stepping frequency (Hz) (SAFE DEFAULT)

// Pulse width for STEP HIGH in microseconds
const unsigned int STEP_PULSE_US = 4;

// A small position-to-speed gain so large positional errors move faster
// Lowered for initial safe testing; increase for faster response after tuning
const double POSITION_SPEED_GAIN = 10.0; // scales step frequency vs step error (SAFE DEFAULT)

// Software limits for position (mm) - adjust to your mechanism travel limits
const double MIN_POSITION_MM = -100.0;
const double MAX_POSITION_MM = 100.0;

// ===== STEPPING TIMER SELECTION =====
// Using HardwareTimer TIM6 at 10 kHz for fast asynchronous stepping
HardwareTimer *tim6_stepper = nullptr;
const unsigned int TIM6_ISR_FREQ_HZ = 10000;  // 10 kHz (100 µs intervals) for fast stepping
// ====================================

void stepperService();


// Compute the target step index from PID output (PIDOut is in mm here)
inline void updateTargetFromPID(double pid_mm) {
  // Center the mapping so PIDOut=0 => no move from current reference
  long deltaSteps = (long)round(pid_mm * STEPS_PER_MM);
  long desired = currentPositionSteps + deltaSteps;
  // Constrain to software limits (convert mm limits to steps)
  long minSteps = (long)floor(MIN_POSITION_MM * STEPS_PER_MM);
  long maxSteps = (long)ceil(MAX_POSITION_MM * STEPS_PER_MM);
  if (desired < minSteps) desired = minSteps;
  if (desired > maxSteps) desired = maxSteps;
  targetPositionSteps = desired;
}

// Convert degrees (mechanical) to microsteps
inline long degToSteps(double deg) {
  double microstepsPerRev = STEPS_PER_REV * MICROSTEPS;
  return (long)round(microstepsPerRev * (deg / 360.0));
}


// TIM6 ISR callback for stepping (called by HardwareTimer at 10 kHz)
// This will be registered with the HardwareTimer instance during setup
void tim6StepperISR() {
  stepperService();  // Run stepping every 100 µs (10 kHz)
}



// Non-blocking stepper service (acts like an interrupt-driven generator)
// Call this frequently (every loop) to produce STEP pulses and update currentPositionSteps
void stepperService() {
  // Do nothing while driver is disabled to avoid generating STEP pulses
  if (digitalRead(ENABLE_PIN) == HIGH) {
    stepState = false;
    return;
  }
  // compute how many steps remain
  long errorSteps = targetPositionSteps - currentPositionSteps;
  if (errorSteps == 0) {
    // nothing to do
    stepState = false;
    return;
  }

  // Decide direction pin (inverted: LOW for forward, HIGH for backward)
  bool dirForward = (errorSteps > 0);
  digitalWrite(DIR_PIN, dirForward ? LOW : HIGH);

  // Map magnitude of error to a stepping frequency
  unsigned long freq = (unsigned long)min((double)STEP_FREQ_MAX, max((double)STEP_FREQ_MIN, POSITION_SPEED_GAIN * fabs((double)errorSteps)));

  // half period in microseconds
  unsigned long halfPeriodUs = (unsigned long)(1000000.0 / (2.0 * freq));

  // Ensure the pulse high time is at least STEP_PULSE_US. We use halfPeriodUs
  // as the high-time for simplicity; enforce a minimum so the driver sees a
  // valid STEP pulse.
  if (halfPeriodUs < STEP_PULSE_US) halfPeriodUs = STEP_PULSE_US;

  unsigned long now = micros();

  if ((now - lastStepTimeUs) < halfPeriodUs) return; // not time yet

  lastStepTimeUs = now;

  if (!stepState) {
    // rising edge
    digitalWrite(STEP_PIN, HIGH);
    stepState = true;
  } else {
    // falling edge completes the step
    digitalWrite(STEP_PIN, LOW);
    stepState = false;

    // update step counter on step completion
    if (dirForward) currentPositionSteps++;
    else currentPositionSteps--;
  }
}

// (Timer-based ISR removed.) The code uses `stepperService()` as the software
// step generator called from `loop()`.



/*
  Notes on interrupts, homing/indexing, and alternatives:

  1) Current implementation uses a non-blocking software "service" driven
     by `micros()` called from the main loop. It behaves similarly to a
     timer interrupt: it toggles the STEP pin on a half-period schedule and
     updates `currentPositionSteps` when a full step pulse completes.

  2) Hardware timer interrupt (recommended for precise timing):
     - On STM32 (Arduino core) you can use `HardwareTimer` (e.g. `HardwareTimer timer(TIM2);`).
     - Configure the timer to fire at the half-period rate (or full-step rate and toggle in ISR).
     - In the ISR, toggle STEP pin and update `currentPositionSteps` on the falling edge.
     - Keep ISR short: only toggle pins and update counters. Do heavier logic in the main loop.

     Example outline (pseudo):
       HardwareTimer timer(TIM2);
       void stepISR() {
         // toggle step pin state and update counter on falling edge
       }
       // in setup: timer.attachInterrupt(stepISR);
       // timer.setOverflow(halfPeriodMicros, MICROSEC_FORMAT);

  3) Indexing / Homing:
     - For reliable absolute positioning you must establish a reference index (home).
     - Use a mechanical limit switch (wired to a GPIO with EXTI) or use the VL53L4CD
       sensor (or other sensor) as an index trigger.
     - When the home sensor fires, set `currentPositionSteps = knownHomeStep` and
       optionally `targetPositionSteps = currentPositionSteps` to re-zero the controller.

     Example EXTI handler outline:
       void homeISR() {
         // disable stepping if needed, set currentPositionSteps = 0;
       }

  4) PID tuning and units:
     - This code treats `PIDOut` as a length (mm). You can instead make the PID
       output be desired step offset directly by changing gains and scaling.
     - The outer loop (VL53 reading -> PID) runs at the sensor update rate. The
       stepper service acts as the inner actuator loop.

  5) Safety and limits:
     - Add software limits on `targetPositionSteps` to avoid running off the ends:
         targetPositionSteps = constrain(targetPositionSteps, minSteps, maxSteps);
     - Add an ESTOP input or monitor stall conditions (via TMC2209 diagnostics over UART)
       if you need closed-loop fault detection.

*/


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
  // Tell the library which hardware pin controls the driver's ENABLE (active low)
  stepper.setHardwareEnablePin(ENABLE_PIN);
  stepper.setRunCurrent(100);
  // stepper.enableAutomaticCurrentScaling();
  // stepper.enableAutomaticGradientAdaptation();
  stepper.enableCoolStep();
  // Don't call stepper.enable() yet — keep driver disabled via ENABLE_PIN
  // until we finish initialization and a short safety delay.

  Serial.begin(115200);
  delay(300);
  Serial.println("time_ms,distance_mm,PIDOut,error");

  // Initialize GPIO pins for TMC2209
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Set initial states
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);     // default direction
  digitalWrite(ENABLE_PIN, HIGH);   // DISABLE the driver at startup (active low)

  // initialize index/reference position to current (user should home for absolute mapping)
  currentPositionSteps = 0;
  targetPositionSteps = 0;
  lastStepTimeUs = micros();
  lastTime = millis();

  // Clear PID integrators for safe startup
  totalError = 0;
  previousError = 0;


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

  // Safety: enable driver after a short delay to ensure no unexpected motion.
  Serial.println("Driver will be enabled in 2000 ms if position is at mechanical zero...");
  delay(2000);
  // The system assumes the mechanism is already at mechanical zero before power-on.
  // Enable the TMC driver now (active low)
  digitalWrite(ENABLE_PIN, LOW);
  stepper.enable();
  // Configure microstepping to 16 (2^4) via the TMC2209 UART API
  stepper.setMicrostepsPerStepPowerOfTwo(4); // 2^4 = 16 microsteps
  Serial.print("Driver enabled. Microsteps: ");
  Serial.println(stepper.getMicrostepsPerStep());
  
  // Initialize TIM6 for 10 kHz stepping via HardwareTimer
  tim6_stepper = new HardwareTimer(TIM6);
  
  // Configure timer: frequency in Hz
  // Formula: freq = timer_clock / (prescaler + 1) / (period + 1)
  // We want 10 kHz, so period should generate that frequency
  uint32_t timerClock = tim6_stepper->getTimerClkFreq();
  uint32_t prescaler = 0;  // No prescaling
  uint32_t period = (timerClock / TIM6_ISR_FREQ_HZ) - 1;
  
  tim6_stepper->setPrescaleFactor(prescaler + 1);
  tim6_stepper->setOverflow(period);
  tim6_stepper->attachInterrupt(tim6StepperISR);
  tim6_stepper->resume();
  
  Serial.println("TIM6 initialized at 10 kHz via HardwareTimer");
  Serial.println("Stepping running asynchronously at 10 kHz (100 µs intervals)");

  Serial.println("Setup complete.");
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
    distance_mm = results.distance_mm + 1;
    range_status = results.range_status;

    // delay(50);
    // // Get measurement results
    // sensor.VL53L4CD_GetResult(&results);

    // // Clear interrupt to allow next measurement
    // sensor.VL53L4CD_ClearInterrupt();

    // // Extract distance and status
    // distance_mm2 = results.distance_mm + 1;
    // range_status2 = results.range_status;

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
    //double e = (distance_mm2 - distance_mm) / 0.05;
    totalError += (e * dt);
    changeError = (e - previousError) / dt;    
    
    // if (e = 0 && distance_mm2 < minDist) {
    //   e = -100;
    // } else if (e = 0 && distance_mm2 > maxDist) {
    //   e = 100;
    // }
    PIDOut = 0.0002 * ((kP * e) + (kI * totalError) + (kD * changeError));

    Serial.print(now);
    Serial.print(",");
    Serial.print(distance_mm);
    Serial.print(",");
    Serial.print(PIDOut, 6);
    Serial.print(",");
    Serial.println(e);  
    // Map PID output (interpreted as mm) into a target step position
    // NOTE: tune kP/kI/kD so that PIDOut is in mm units (or scale here)
    updateTargetFromPID(PIDOut);

    previousError = e;
    lastTime = now;
   }


    // Non-blocking serial command interface
    // if (Serial.available()) {
    //   char c = (char)Serial.read();
    //   if (c == '\r' || c == '\n') {
    //     // ignore
    //   } else {
    //     switch (c) {
    //       case '0':
    //         targetPositionSteps = 0;
    //         Serial.println("CMD: target -> 0 degrees");
    //         break;
    //       case '1':
    //         targetPositionSteps = degToSteps(45.0);
    //         Serial.println("CMD: target -> 45 degrees");
    //         break;
    //       case '+':
    //         targetPositionSteps += degToSteps(1.0);
    //         Serial.print("CMD: increment target by 1 deg -> "); Serial.println(targetPositionSteps);
    //         break;
    //       case '-':
    //         targetPositionSteps -= degToSteps(1.0);
    //         Serial.print("CMD: decrement target by 1 deg -> "); Serial.println(targetPositionSteps);
    //         break;
    //       case 'h':
    //         currentPositionSteps = 0;
    //         targetPositionSteps = 0;
    //         Serial.println("CMD: homed - current position set to 0");
    //         break;
    //       case 's':
    //         Serial.print("STATUS: currentSteps="); Serial.print(currentPositionSteps);
    //         Serial.print(", targetSteps="); Serial.println(targetPositionSteps);
    //         break;
    //       case 'i':
    //         Serial.println("No hardware ISR in use; 'i' not available.");
    //         break;
    //       case 'p':
    //         // Manual single step (software) for debugging. Requires driver enabled.
    //         if (digitalRead(ENABLE_PIN) == HIGH) {
    //           Serial.println("Driver disabled - enable before pulsing");
    //         } else {
    //           // choose direction based on sign of (target-current) if any, otherwise forward
    //           bool dirForward = (targetPositionSteps - currentPositionSteps) >= 0;
    //           digitalWrite(DIR_PIN, dirForward ? HIGH : LOW);
    //           noInterrupts();
    //           digitalWrite(STEP_PIN, HIGH);
    //           delayMicroseconds(STEP_PULSE_US);
    //           digitalWrite(STEP_PIN, LOW);
    //           if (dirForward) currentPositionSteps++; else currentPositionSteps--;
    //           interrupts();
    //           Serial.print("Manual step, now currentSteps="); Serial.println(currentPositionSteps);
    //         }
    //         break;
    //       default:
    //         Serial.print("Unknown command: "); Serial.println(c);
    //         Serial.println("Commands: 0=0deg,1=45deg,+/-=±1deg,h=home,s=status");
    //         break;
    //     }
    //   }
    // }
    // SysTick-driven stepping runs independently; no stepper call here
  // Software stepping driven by SysTick timer (independent of loop)
}
