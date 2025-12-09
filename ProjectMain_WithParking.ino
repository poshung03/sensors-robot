#include <Arduino.h>
#include <SoftPWM.h>
#include <string.h>
#include "SunFounder_AI_Camera.h"
#include <Servo.h>
#include "follow.cpp"
#include "park.cpp"
#include "avoid.cpp"


// DEFINE MODES

#define MODE_DEFAULT 0
#define MODE_PARK 1
#define MODE_RC 2
#define MODE_FOLLOW 3
#define MODE_AVOID 4

uint8_t currentMode = MODE_RC;

int timer = 0;


// Create a Servo object
Servo servo;

// Wifi configuration and credentials
// In AP Mode
#define WIFI_MODE WIFI_MODE_AP
#define SSID "BetterBot"
#define PASSWORD "12345678"

// Device configuration
#define NAME "GalaxyRVR"  // Device name
#define TYPE "AiCamera"   // Device type
#define PORT "8765"       // Port for the SunFounder Controller APP



#define SERVO_PIN 6
#define SERVO_REVERSE false

// Define the pin for the ultrasonic module
#define ULTRASONIC_PIN 10

// Define the pins for the IR modules
#define IR_RIGHT 7
#define IR_LEFT 8

// This pin reads the voltage of the battery
#define BATTERY_PIN A3


/** Set the pins for the motors */
#define MOTOR_PINS \
  (uint8_t[4]) { \
    2, 3, 4, 5 \
  }
/** Set the positive and negative directions for the motors */
#define MOTOR_DIRECTIONS \
  (uint8_t[2]) { \
    0, 1 \
  }

// Define the pins of motors
const int in1 = 2;
const int in2 = 3;
const int in3 = 4;
const int in4 = 5;

/* variables of motors and servo*/
int8_t leftMotorPower = 0;
int8_t rightMotorPower = 0;
uint8_t servoAngle = 90;

// Simple move functions
void carForward(int8_t power) {
  carSetMotors(power, power);
}
void carBackward(int8_t power) {
  carSetMotors(-power, -power);
}
void carTurnLeft(int8_t power) {
  carSetMotors(-power, power);
}
void carTurnRight(int8_t power) {
  carSetMotors(power, -power);
}
void carStop() {
  carSetMotors(0, 0);
}


/* variable of esp32-cam flash lamp*/
bool cam_lamp_status = false;


// Create an AiCamera object
AiCamera aiCam = AiCamera(NAME, TYPE);

/* Config Camera Servo */
Servo myServo;

void setup() {
  int m = millis();
  Serial.begin(115200);
  Serial.print("GalaxyRVR version ");

  Serial.println(F("Initializing..."));

  // Attach the servo on pin 6
  myServo.attach(6);

  // Initialize the AiCamera
  aiCam.begin(SSID, PASSWORD, WIFI_MODE, PORT);
  // Set the function to execute when data is received
  aiCam.setOnReceived(onReceive);
  // Set the command timeout
  aiCam.setCommandTimeout(100);

  //set IR module pins as inputs
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);

  // Initialize SoftPWM
  SoftPWMBegin();
}

void loop() {
  aiCam.loop();
  modeHandler();
}







void onReceive() {
  //Perform all data readings from the sensors and update positions of everything
  dataRead();

  //MODE SELECTION
  if (aiCam.getSwitch(REGION_E)) {
    if (currentMode != MODE_AVOID) {
      currentMode = MODE_AVOID;
      Serial.println("Switched to Obstacle Avoidance");
    }
  } else if (aiCam.getSwitch(REGION_G)) {
    if (currentMode != MODE_PARK) {
      currentMode = MODE_PARK;
      Serial.println("Switched to Park Mode");
    }
  } else if (aiCam.getSwitch(REGION_F)) {
    if (currentMode != MODE_FOLLOW) {
      currentMode = MODE_FOLLOW;
      Serial.println("Switched to Obstacle Following");
    } else {
      if (currentMode = !MODE_RC) {
        currentMode = MODE_RC;
      }
    }
  }

// throttle
  int throttle_L = aiCam.getThrottle(REGION_K);
  int throttle_R = aiCam.getThrottle(REGION_Q);
  // Serial.print("throttle_L: "); Serial.print(throttle_L);
  // Serial.print("throttle_R: "); Serial.println(throttle_R);
  if (throttle_L != 0 || throttle_R != 0 || throttle_L != leftMotorPower || throttle_R != rightMotorPower) {
    currentMode = MODE_RC;
    leftMotorPower = throttle_L;
    rightMotorPower = throttle_R;
  }

}



void modeHandler() {
  switch (currentMode) {
    case MODE_DEFAULT:
      timer = 0;
      carStop();
   //   servoAngle = 90;
      servo.write(servoAngle);
      Serial.println("In mode default"); 
      break;
    case MODE_RC:
      timer = 0;
      carStop();
      servo.write(servoAngle);
      carSetMotors(leftMotorPower, rightMotorPower);
      Serial.println("In mode RC"); 
      break;
    case MODE_PARK:
      park();
      break;
    case MODE_FOLLOW:
      carStop();
  //    follow();
      break;
  //  case MODE_AVOID:
      //avoid();
  //    break;
  }
}


// Function to set the power of the motors
void carSetMotors(int8_t power_L, int8_t power_R) {
  // Set power for the left motor
  if (power_L >= 0) {
    SoftPWMSet(in1, map(power_L, 0, 100, 0, 255));
    SoftPWMSet(in2, 0);
  } else {
    SoftPWMSet(in1, 0);
    SoftPWMSet(in2, map(power_L, 0, -100, 0, 255));
  }

  // Set power for the right motor
  if (power_R >= 0) {
    SoftPWMSet(in3, 0);
    SoftPWMSet(in4, map(power_R, 0, 100, 0, 255));
  } else {
    SoftPWMSet(in3, map(power_R, 0, -100, 0, 255));
    SoftPWMSet(in4, 0);
  }
}

float readSensorData() {
  // A 4ms delay is required, otherwise the reading may be 0
  delay(4);

  //Set to OUTPUT to send signal
  pinMode(ULTRASONIC_PIN, OUTPUT);

  // Clear the trigger pin
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by sending a high pulse for 10us
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);

  // Set the trigger pin back to low
  digitalWrite(ULTRASONIC_PIN, LOW);

  //Set to INPUT to read
  pinMode(ULTRASONIC_PIN, INPUT);

  // pulseIn returns the duration of the pulse on the pin
  float duration = pulseIn(ULTRASONIC_PIN, HIGH);

  // Calculate the distance (in cm) based on the speed of sound
  //(340 m/s or 0.034 cm/us)
  float distance = duration * 0.034 / 2;

  return distance;
}


void dataRead() {
  // Get the value of the slider in region D
  int16_t sliderD = aiCam.getSlider(REGION_D);

  // Move the servo to the angle indicated by the slider
  myServo.write(int(sliderD));

  // Get the throttle values for the left and right
  int throttle_L = aiCam.getThrottle(REGION_K);
  int throttle_R = aiCam.getThrottle(REGION_Q);

  // Set the power for the motors
  carSetMotors(throttle_L, throttle_R);

  // Read values from IR sensors
  int leftValue = digitalRead(IR_LEFT);
  int rightValue = digitalRead(IR_RIGHT);
  aiCam.sendDoc["N"] = leftValue;
  aiCam.sendDoc["P"] = rightValue;

  // ultrasonic
  float distance = readSensorData();
  aiCam.sendDoc["O"] = distance;
}

void park(){
  //below is the meneuver once a proper spot has been found
  //carForward(100);
 // delay(100);
  //carBackward(100);
  //delay(100);
  //So, the IR sensors don't really work too well when the object is moving continuously,
  //So we have to stop intermittently and then measure rightValue and leftValue; this is essentially
  //clocking the reading of rightValue and leftValue
  //The more often it stops, the more accurate it will be
  //WE'LL have it read the values of 
  int rightValue = digitalRead(IR_RIGHT);
  int leftValue = digitalRead(IR_LEFT);

  carStop();
  delay(2000);
  if (rightValue == 1){
    timer = timer + 1;
  }
  Serial.println(rightValue);
  carForward(20);
  delay(500);
  if (rightValue == 0){

    if (timer > 5) {
      carSetMotors(-45,0);
      delay(1000);
      carSetMotors(0,0);
      delay(1000);
      carBackward(20);
      delay(1000);
    // carBackward(100);
      carSetMotors(0,-45);
      delay(1000);
      carSetMotors(0,0);
      carForward(20);
      delay(1000);
      carBackward(20);
      delay(1000);
      carStop();
      delay(10000);
      timer = 0;
  }
  else{
    timer = 0;
  }
  }
}
