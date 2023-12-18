#include "MPU9250.h"

#define DIR 7
#define BRAKE 8
#define PWM 3

double dt, last_time;
double input, output, integral, previous, error;
const double Setpoint = 3.5;

//Specify the links and initial tuning parameters
double kp = 40, ki = 0.5, kd = 2;  //38-0.5-2 or 40-0.5-2   100-5-10  130-6-12

MPU9250 mpu;

void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000001;

  Serial.begin(115200);
  Wire.begin();

  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(BRAKE, OUTPUT);

  input = mpu.getPitch();

  digitalWrite(BRAKE, HIGH);
  analogWrite(PWM, 0);

  delay(2000);

  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with connection_check example.");
      delay(5000);
    }
  }
}

void loop() {

  input = mpu.getPitch();

  double now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;

  input = mpu.getPitch();


  error = Setpoint - input;
  output = pid(error);

  if (output < 0) {
    digitalWrite(DIR, HIGH);
    output = -output;
  } 
  else 
    digitalWrite(DIR, LOW);


  if (output >= 255) {
    output = 255;
  }

  output = map(output, 0, 255, 255, 0);

  TCCR2B = TCCR2B & B11111000 | B00000001;
  analogWrite(PWM, output);

  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      print();
      prev_ms = millis();
    }
  }
}


void print() {
  Serial.print("Pitch angle: ");
  Serial.print(input, 2);
  Serial.print("    ;error: ");
  Serial.print(error, 2);
  Serial.print("    ;output: ");
  Serial.println(output, 2);
}

double pid(double error) {

  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}