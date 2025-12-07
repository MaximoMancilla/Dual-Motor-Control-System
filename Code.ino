/* ============================================================
    Project: Dual Motor Control System (ESP32)
    Author:  Máximo Mancilla
    Date:    2025
    Description:
      Two buttons select motor direction (left / right),
      and a potentiometer controls speed via PWM.
      Both TT motors run through an H-bridge driver.
   ============================================================ */

/// --------------------- INPUT PINS --------------------------
int leftBut  = 0;
int rightBut = 4;
int pot      = 34;

/// ------------------- MOTOR DRIVER PINS ---------------------
int pwmMA = 22;   // Motor A PWM
int in1   = 25;
int in2   = 26;

int pwmMB = 21;   // Motor B PWM
int in3   = 27;
int in4   = 14;

/// ------------------ PWM CONFIGURATION ----------------------
int freq       = 5000;
int resolution = 8;
int channelA   = 0;
int channelB   = 1;

/// ---------------------- VARIABLES --------------------------
int leftVal;
int rightVal;
int potVal;

int motorSpeed;
int direction = 0;       // -1 = left, 1 = right, 0 = stopped
int minPWM    = 90;
int maxPWM    = 255;

/// ============================================================
///                         SETUP
/// ============================================================
void setup() {

  // Buttons with internal pullups
  pinMode(leftBut,  INPUT_PULLUP);
  pinMode(rightBut, INPUT_PULLUP);

  // Motor direction pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Attach PWM to pins using the modern ESP32 function
  ledcAttach(pwmMA, freq, resolution);
  ledcAttach(pwmMB, freq, resolution);

  Serial.begin(115200);
  Serial.println("=== Dual Motor Control System Started ===");
}

/// ============================================================
///                          LOOP
/// ============================================================
void loop() {

  // Read inputs
  leftVal  = digitalRead(leftBut);
  rightVal = digitalRead(rightBut);
  potVal   = analogRead(pot);

  // Speed control with dead zone
  if (potVal < 200) {
    motorSpeed = 0;
  } else {
    motorSpeed = map(potVal, 200, 4095, minPWM, maxPWM);
  }

  // Direction control (buttons override)
  if (leftVal  == LOW) direction = -1;
  if (rightVal == LOW) direction = 1;

  // Apply direction to both motors
  if (direction == -1) {         // LEFT
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (direction == 1) {     // RIGHT
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {                         // STOP
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Send PWM to motors
  ledcWrite(pwmMA, motorSpeed);
  ledcWrite(pwmMB, motorSpeed);

  // Debug output
  Serial.print("Pot: ");   Serial.print(potVal);
  Serial.print(" | Speed: "); Serial.print(motorSpeed);
  Serial.print(" | Dir: ");   Serial.println(direction);
}
