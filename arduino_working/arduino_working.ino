//*******************************************************************//
//                          DEFINITIONS                              //
//*******************************************************************//

// #define EN_A 6
#define IN1 11
#define IN2 10

// #define EN_B 5
#define IN3 9
#define IN4 6

// #define Ultrasound sensor pins
#define TRIGPIN 12
#define ECHOPIN 13

//*******************************************************************//
//                        GLOBAL CONSTANTS                           //
//*******************************************************************//

// Ultrasound sensor delay limits
const long max_us = 23000;
const long min_us = 100;

// PID constants
const float Kp = 0.01;     // Proportional gain
const float Ki = 0.00005;  // Integral gain
const float Kd = 0.00000;  // Derivative gain

//*******************************************************************//
//                    Global counting variables                      //
//*******************************************************************//

// PID loop variables
float x_target = 0.0;      // Expected distance from wall
float integral = 0.0;      // Current value of PID integral
float dx_prev = 0.0;       // Previous deviation
float dt = 0;              // Time between loops
unsigned long prev_time = 0;

// Turning control
bool turned = false;       // Completed the first turn

// Obsolete:
// bool should_stop = false;  // Completed the second turn

//*******************************************************************//






//*******************************************************************//
//                         Motor Control                             //
//*******************************************************************//

/*
void throttle_motor(uint8_t value):

  Sets the speed of the driving motor forward.

  param value: 8 bit value of the desired motor speed.
*/
void throttle_motor(uint8_t value) {
  analogWrite(IN3, 0);
  analogWrite(IN4, value);
}

/*
void reverse(uint8_t value):

  Sets the speed of the driving motor backward.

  param value: 8 bit value of the desired motor speed.
*/
void reverse(uint8_t value) {
  analogWrite(IN3, value);
  analogWrite(IN4, 0);
}

/*
void yaw_motor(bool right, uint8_t value):

  Sets the speed of the yaw motor.

  param right: Whether or not the robot should turn right.
  param value: 8 bit value of the desired motor speed.
*/
void yaw_motor(bool right, uint8_t value) {
  if (right) {
    analogWrite(IN1, 0);
    analogWrite(IN2, value);
  } else {
    analogWrite(IN1, value);
    analogWrite(IN2, 0);
  }
}

/*
void motors_off():

  Sets all motors to speed 0, stopping them.
*/
void motors_off() {
  throttle_motor(0);
  yaw_motor(true, 0);
}

//*******************************************************************//
//                       Ultrasonic sensor                           //
//*******************************************************************//

/*
long ultrasound_dist():

  Runs the ultrasonic sensor once to find a singular distance,
  which is returned in terms of the duration of pulse measured
  by the ultrasonic sensor.

  return: Returns the measured duration of the pusle as a long.
*/
long ultrasound_dist() {
  long duration;

  // Send pulse
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  // Measure time to response.
  duration = pulseIn(ECHOPIN, HIGH, max_us + 100);
  if (duration > max_us || duration < min_us) {    // Check that the signal is within bounds.
    return -1;
  } else {
    return duration;
  }
}

/*
void measure_position():

  Wraps ultrasound_dist() to calculate the average measured
  distance over ten cycles and then sets that as the new
  expected distance (x_target).
*/
void measure_position() {
  int loops = 10;   // Set number of samples
  long total = 0;

  for(int i = 0; i < loops; i++) {
    long duration = ultrasound_dist();
    if (duration == -1) {   // Resample if measurement was out of bounds.
      i--;
    }

    total += duration;
  }

  x_target = total / loops;
}

//*******************************************************************//
//                              PID                                  //
//*******************************************************************//

/*
float pid_update(float x):

  Calculates necessary steering correction using a PID
  (Proportional-Integral-Derivative) controller setup.
  The impact of each term in the PID loop is set by the
  PID constants at the begining of this file for ease
  of access.

  param x: Current distance measured by the ultrasonic sensor.
  return: Calculated steering correction as a float.
*/
float pid_update(float x) {
  float dx = x_target - x;

  float dx_int;                        // Deviation from expected distance to wall.
  if (dx < 25) {                       // Small margin to account for variation in sensor data.
    dx_int = 0;
  } else {
    dx_int = dx;
  }

  integral += dx_int * dt;            // Accumulate the error.
  float ddx = (dx - dx_prev) / dt;    // Derivative term.
  dx_prev = dx;

  return Kp * dx + Ki * integral + Kd * ddx;
}

//*******************************************************************//




//*******************************************************************//
//                        ARDUINO CONTROL                            //
//*******************************************************************//

void setup() {
  pinMode(TRIGPIN, OUTPUT);  // Sets the TRIGPIN as an Output.
  pinMode(ECHOPIN, INPUT);   // Sets the ECHOPIN as an Input.
  Serial.begin(115200);      // Starts the serial communication.

  measure_position();        // Measures initial distance from wall.
}

void loop() {
  // Update time variables.
  dt = millis() - prev_time;
  prev_time = millis();


  throttle_motor(255);

  // Measure distance from wall.
  long duration = ultrasound_dist();
  if (duration == -1) {      // Filters out of bound returns.
    return;
  }

  // Print statements for debugging purpuses.
  Serial.print("Duration: ");
  Serial.print(duration);

  bool turning_point = duration > 4000;  // Detect wall out of range.

  if (!turning_point) {
    float out = pid_update((float)duration);
    float steering_correction = constrain(out, -50.0, 50.0);

    // Print statements for debugging purpuses.
    Serial.print("  Correction: ");
    Serial.println(steering_correction);

    bool direction = steering_correction > 0;   // Positive steering correction means away from the wall.

    if(abs(steering_correction) > 1){   // Ensures that the robot does not correct within a margin of error.
      yaw_motor(direction, constrain(abs(steering_correction) * .5, 0, 255));
    }

  } else if (!turned) {     // Frist turn
    delay(500);             // Delay to approach the wall before turning.
    throttle_motor(0);
  
    yaw_motor(true, 255);   // Turning
    delay(600);

    motors_off();           // Grace period to ensure robot comes to a stop.
    delay(355);

    throttle_motor(255);
    delay(350);
    
    measure_position();     // Set a new expected distance from the wall.
    turned = true;
  }
  else {                    // Second turn
    reverse(255);           // We use reverse here so that the robot stops faster.
    delay(200);
    reverse(0);
    delay(50);              // Grace period to ensure a complete stop.

    yaw_motor(false, 255);  // Turning
    delay(500);
    motors_off();

    throttle_motor(255);    // Move motor forward until it is at the wall again.
    delay(4000);

    motors_off();

    delay(50000);           // Delay to grab the robot.

        // Obsolete:
    // if(should_stop) {
    //   motors_off();
    //   delay(10000);
    // } else {
    //   should_stop = true;
    // }
  }

  delay(50);
  yaw_motor(true, 0);
}