#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>


#include <Servo.h>

#include <Wire.h>
#include <MPU6050.h>
#include "h.h"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO, BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
int lastNumber = -1;

Servo myservo;  // DO NOT REMOVE
int pulse = 1500;

MPU6050 mpu;

char charBuffer[3];
double aVar = 0;

// lean error
unsigned long lastTime;
double gyroAngle;
double gyroAngleDelta;
double tilt;
int16_t ax, ay, az, gx, gy, gz;
double PID_Output = 0;
double integral = 0;
double prevError = 0;
double prevPID = 0;
unsigned long currentTime;
float deltaTime;
double angle_error;
double derivative;
double PID_output;
int motorSpeed;
double targetAngle = 0;


void error(const __FlashStringHelper* err) {
  Serial.println(err);
  while (1)
    ;
}

void setup(void) {
  while (!Serial)
    ;
  Serial.begin(115200);

  if (!ble.begin(true)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if (!ble.setMode(BLUEFRUIT_MODE_DATA)) {
    error(F("Failed to set BLE MODE DATA"));
  }

  ////mpu initialization
  Wire.begin();
  mpu.initialize();

  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM, OUTPUT);
  // // analogWrite(PWM, 0);


  pinMode(DIR_A2, OUTPUT);
  pinMode(DIR_B2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  // analogWrite(PWM2, 0);


  // pinMode(servoPWM, OUTPUT);
  myservo.attach(servoPWM);

  lastTime = millis();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyroAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  // mpu connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }
  Serial.println("MPU6050 good");

  //Startup from bottom
  // for (int i = 1; i <= 25; i++) {
  //   digitalWrite(DIR_A2, LOW);
  //   digitalWrite(DIR_B2, HIGH);
  //   analogWrite(PWM2, i*25);
  //   delay(i*10);
  //   digitalWrite(DIR_A2, HIGH);
  //   digitalWrite(DIR_B2, LOW);
  //   delay(i*10);
  // }
}

void loop() {
  bt();

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //tilt
  tilt = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI - BALANCE_EQUIL;

  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  gyroAngleDelta = gy / 100 * deltaTime;
  if (isnan(gyroAngle)) gyroAngle = tilt;
  else gyroAngle = 0.98 * (gyroAngle - gyroAngleDelta) + 0.02 * tilt; //98 02

  targetAngle = (BALANCE_EQUIL-gyroAngle) / 3 - aVar;

  // Calculate PID output
  // pd
  //.6 .064 .0
  const double Kp = .5;    //p 
  const double Kd = 0.1;    //d .1
  const double Ki = 0.05;  //0//0.05

  angle_error = targetAngle - gyroAngle;
  integral += angle_error * deltaTime;
  if (!((angle_error > 0) & (prevError > 0)) | !((angle_error < 0) & (prevError < 0))) integral = 0;
  derivative = (angle_error - prevError) / deltaTime;
  prevError = angle_error;
  PID_output = Kp * angle_error + Kd * derivative + Ki * integral;

  motorSpeed = 0;
  if (abs(angle_error) <= 20) {
    motorSpeed = exponentialMap(abs(PID_output), ERRORMIN, ERRORMAX, MOTORMIN, MOTORMAX);
    motorSpeed = constrain(motorSpeed, MOTORMIN, MOTORMAX);
  }

  RW_en(PID_output, motorSpeed);
  printVals(targetAngle, gyroAngle, PID_output, motorSpeed);
}

void manualWrite(int pin) {
  for (int i = 0; i < 2; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(pin, LOW);
    delay(20 - (pulse / 1000));
  }
}

float exponentialMap(float x, int in_min, int in_max, int out_min, int out_max) {
  // Map x from in_min-in_max to a 0-1 range
  float normalized = float(x - in_min) / float(in_max - in_min);

  // Apply exponential curve - adjust exponent as needed
  float exponential = pow(normalized, 2);  // Square the normalized value for a parabolic effect

  // Map the exponential value to the desired output range
  return exponential * (out_max - out_min) + out_min;
}

void bt(){
    if (ble.available()) {
    ble.readBytesUntil('!', charBuffer, 3);
    switch (charBuffer[2]) {
      case '1':
        // Serial.print("Forward\n");
        digitalWrite(DIR_A, HIGH);
        digitalWrite(DIR_B, LOW);
        analogWrite(PWM, 255);
        break;
      case '2':
        // Serial.print("Backward\n");
        digitalWrite(DIR_A, LOW);
        digitalWrite(DIR_B, HIGH);
        analogWrite(PWM, 255);
        break;
      case '0':
        // Serial.print("Stop\n");
        digitalWrite(DIR_A, LOW);
        digitalWrite(DIR_B, LOW);
        analogWrite(PWM, 0);
        break;
      default:
        // Serial.println("Default driving");
        break;
    }
    // Serial.println(charBuffer[0]);
    // switch (charBuffer[0]) {
    //   case '0':
    //     // Serial.println("center");
    //     pulse = 1500;
    //     break;
    //   default:
    //     // Serial.println("Default steering");
    //     pulse = map(charBuffer[0] - '0', 1, 9, 1500, 1600);  //600 2500
    //     // aVar = map(pulse, 1000, 1900, -15, 15);
    //     break;
    // }
     if (strncmp(charBuffer, "00", 2) == 0) {
        pulse = 1500;
      }else {
      char servoBuffer[2] = {charBuffer[0], charBuffer[1]};  
    int servoValue = atoi(servoBuffer);
    pulse = map(servoValue, 01, 99, 2200, 800);
      }
  }
  manualWrite(servoPWM);
}

void RW_en(double pid, int motorSpeed){
  if (pid > 0) {  //Check direction
    digitalWrite(DIR_A2, LOW);
    digitalWrite(DIR_B2, HIGH);
  } else {
    digitalWrite(DIR_A2, HIGH);
    digitalWrite(DIR_B2, LOW);
  }
  analogWrite(PWM2, motorSpeed);
}

void printVals(double targetAngle, double gyroAngle, double PID_output, int motorSpeed){
    //PRINTING VALUES

  // Serial.print("Tilt:");      // print value label
  // Serial.print(tilt);         // print x-axis acceleration
  // Serial.print(", ");

  // Serial.print("avg:");
  // Serial.print(avgTilt);         // print y-axis acceleration
  // Serial.print(", ");

  Serial.print("TARGETANG:");
  Serial.print(targetAngle);
  Serial.print(", ");
  Serial.print("GyroAngle:");
  Serial.print(gyroAngle);
  Serial.print(", ");

  Serial.print("PID:");
  Serial.print(PID_output);
  Serial.print(", ");

  Serial.print("mS:");       // print value label
  Serial.print(motorSpeed);  // print x-axis acceleration
  Serial.println(", ");

  // Serial.print("Speed:");      // print value label
  // Serial.print((avgTilt - prev_tilt) * 10);         // print x-axis acceleration
  // Serial.print(", ");

  // Serial.print("gx:");
  // Serial.print(gx);         // print y-axis acceleration
  // Serial.print(", ");

  // Serial.print("gy:");
  // Serial.println(gy);         // print y-axis acceleration
  // Serial.print(", ");

  // Serial.print("target:");
  // Serial.print(targetAngle);         // print y-axis acceleration
  // Serial.print(", ");

  // Serial.print("desired:");
  // Serial.println(gyroAngle);         // print y-axis acceleration
  // Serial.print(", ");

  // Serial.print("gz:");
  // Serial.println(gz);       // print z-axis acceleration
}