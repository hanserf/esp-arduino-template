/**
   Author Teemu Mäntykallio
   Initializes the library and runs the stepper
   motor in alternating directions.
*/

#include <TMCStepper.h>

#define EN_PIN           4 // Enable
#define DIR_PIN          3 // Direction
#define STEP_PIN         2 // Step
#define SW_RX            0 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            1 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT  Serial // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define MICRO_STEPS 64 // Desired microsteps (1/2, 1/4, 1/8 ...) 
#define VELOCITY 3000 // Currently Delay, TODO: deg/s

#define R_SENSE 0.11f // Match to your driver
// SilentStepStick series use 0.11
// UltiMachine Einsy and Archim2 boards use 0.2
// Panucatt BSD2660 uses 0.1
// Watterott TMC5160 uses 0.075

TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware Serial
//TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial
//TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);


unsigned long time_now = 0;

// ** TMC2209Stepper driver = TMC2209Stepper(&SERIAL_PORT, R_SENSE);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  // Serial.begin(115200);

  // Enable one according to your setup
  //SPI.begin();                    // SPI drivers
  SERIAL_PORT.begin(115200);      // HW UART drivers
  while (!Serial) {
    digitalWrite(13, HIGH);
  }
  digitalWrite(13, LOW);

  //driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 // SPI: Init CS pins and possible SW SPI pins
  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current mA
  driver.microsteps(16);          // Set microsteps to 1/16th

  //driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  //driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
}

bool shaft = false;

int byteStream = 0;


void loop() {
  motorMoveXSteps(20);
  delay(200);
  motorToggleDir(DIR_PIN);
}

void motorMoveXSteps(int steps) {
  time_now = millis();
  bool result = false;
  int cnt = 0;
  while(!result){
    if
    
    
  }
    motorMoveOneStep();
    delay(2);
 
}


void motorMoveOneStep() {
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
}


void motorToggleDir(int pin) {
  if (digitalRead(pin)) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}



void motorSetZero() {
  for (uint16_t i = 4 * 20 * MICRO_STEPS ; i > 0; i--) {
    motorMoveOneStep();
    delayMicroseconds(3000);
  }
}
