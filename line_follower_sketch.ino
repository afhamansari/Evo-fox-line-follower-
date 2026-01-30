/*
 Line Following Code for Robojunkies LF-2 robot with Digital IR Sensor Array
 Modified for 5-channel digital output IR sensors (like TCRT5000L with digital output)
*/

#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

#define BUZZER 11       // Buzzer on digital pin 11
#define START_STOP_BTN 12  // Start/Stop button on pin 12

// Digital sensor pins (connected to analog pins A1-A5, used as digital inputs)
#define SENSOR1 A1  // Leftmost sensor
#define SENSOR2 A2  // Left-center sensor
#define SENSOR3 A3  // Center sensor
#define SENSOR4 A4  // Right-center sensor
#define SENSOR5 A5  // Rightmost sensor

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 255;
int turnSpeed = 200;  // Slower speed for sharp turns

float Kp = 1.0;
float Kd = 1.0;
float Ki = 0.0;

int s1, s2, s3, s4, s5;

bool running = false;  // Line follower running state

void setup()
{
  Serial.begin(9600);

  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(START_STOP_BTN, INPUT_PULLUP);

  // Initialize buzzer off
  digitalWrite(BUZZER, LOW);
}

void loop()
{
  // Check for button press to toggle running state
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(START_STOP_BTN);

  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    // Button pressed, toggle running
    running = !running;

    if (!running)
    {
      // Stop the motors and buzzer when stopping
      motor1.drive(0);
      motor2.drive(0);
      digitalWrite(BUZZER, LOW);
    }

    delay(300); // Debounce delay
  }
  lastButtonState = currentButtonState;

  if (running)
  {
    readSensors();

    // Sharp left turn condition
    if ((s1 == 0 || s2 == 0) && s3 == 1 && s4 == 1 && s5 == 1)
    {
      // Buzz while turning left
      digitalWrite(BUZZER, HIGH);

      // Move forward slightly
      motor1.drive(lfspeed);
      motor2.drive(lfspeed);
      delay(50);

      // Turn left until center sensor detects line
      while (digitalRead(SENSOR3) == 1)
      {
        motor1.drive(0);
        motor2.drive(turnSpeed);
        readSensors();
      }

      // Stop buzzing after turn
      digitalWrite(BUZZER, LOW);
    }
    // Sharp right turn condition
    else if (s1 == 1 && s2 == 1 && s3 == 1 && (s4 == 0 || s5 == 0))
    {
      // Buzz while turning right
      digitalWrite(BUZZER, HIGH);

      // Move forward slightly
      motor1.drive(lfspeed);
      motor2.drive(lfspeed);
      delay(50);

      // Turn right until center sensor detects line
      while (digitalRead(SENSOR3) == 1)
      {
        motor1.drive(turnSpeed);
        motor2.drive(0);
        readSensors();
      }

      // Stop buzzing after turn
      digitalWrite(BUZZER, LOW);
    }
    else
    {
      // Not turning sharply, no buzz
      digitalWrite(BUZZER, LOW);
      linefollow();
    }
  }
  else
  {
    // If not running, motors and buzzer off is ensured above, can add small delay
    delay(100);
  }
}

void readSensors()
{
  s1 = digitalRead(SENSOR1);
  s2 = digitalRead(SENSOR2);
  s3 = digitalRead(SENSOR3);
  s4 = digitalRead(SENSOR4);
  s5 = digitalRead(SENSOR5);
}

void linefollow()
{
  int position = 0;
  int activeCount = 0;

  if (s1 == 0) { position += -2; activeCount++; }
  if (s2 == 0) { position += -1; activeCount++; }
  if (s3 == 0) { position +=  0; activeCount++; }
  if (s4 == 0) { position +=  1; activeCount++; }
  if (s5 == 0) { position +=  2; activeCount++; }

  if (activeCount > 0) {
    error = position;
  } else {
    error = previousError;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  if (I > 100) I = 100;
  if (I < -100) I = -100;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0) lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0) rsp = 0;

  motor1.drive(lsp);
  motor2.drive(rsp);
}
