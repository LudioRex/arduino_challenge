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
const long min_us = 1000;

// PID constants
const float Kp = 0.01;      // Proportional gain
const float Ki = 0.00005;  // Integral gain
const float Kd = 0.00000;  // Derivative gain

//*******************************************************************//


// Global counting variables

float x_target = 0.0;
float integral = 0.0;
float dx_prev = 0.0;
float dt = 0;
unsigned long prev_time = 0;

void throttle_motor(uint8_t value) {
  analogWrite(IN3, 0);
  analogWrite(IN4, value);
}

void weeee_back(uint8_t value) {
  analogWrite(IN3, value);
  analogWrite(IN4, 0);
}

void yaw_motor(bool right, uint8_t value) {
  if (right) {
    analogWrite(IN1, 0);
    analogWrite(IN2, value);
  } else {
    analogWrite(IN1, value);
    analogWrite(IN2, 0);
  }
}

void motors_off() {
  throttle_motor(0);
  yaw_motor(true, 0);
}

long ultrasound_dist() {
  long duration;

  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  duration = pulseIn(ECHOPIN, HIGH, max_us + 100);
  if (duration > max_us) {
    return -1;
  } else if (duration < min_us) {
    return -1;
  } else {
    return duration;
  }
}

// PID loop update
float pid_update(float x) {
  float dx = x_target - x;

  float dx_int;
  if (dx < 25) {
    dx_int = 0;
  } else {
    dx_int = dx;
  }

  integral += dx_int * dt;
  float ddx = (dx - dx_prev) / dt;
  dx_prev = dx;

  return Kp * dx + Ki * integral + Kd * ddx;
}

void measure_position() {
  int loops = 10;
  long total = 0;

  for(int i = 0; i < 10; i++) {
    long duration = ultrasound_dist();
    if (duration == -1) {
      i--;
    }

    total += duration;
  }

  x_target = total / loops;
}

void setup() {
  pinMode(TRIGPIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHOPIN, INPUT);   // Sets the echoPin as an Input
  Serial.begin(115200);      // Starts the serial communication

  measure_position();
}

bool turned = false;
bool should_stop = false;

void loop() {
  // Update time variables.
  dt = millis() - prev_time;
  prev_time = millis();

  throttle_motor(255);

  long duration = ultrasound_dist();
  if (duration == -1) {
    return;
  }

  bool turning_point = duration > 4000;

  Serial.print("Duration: ");
  Serial.print(duration);

  if (!turning_point) {
    float out = pid_update((float)duration);
    float steering_correction = constrain(out, -50.0, 50.0);


    Serial.print("  Correction: ");
    Serial.println(steering_correction);

    bool sign = steering_correction > 0;

    if(abs(steering_correction) > 1){
      yaw_motor(sign, constrain(abs(steering_correction) * .5, 0, 255));
      
    }
  

  } else if (!turned) {
    delay(500);
    throttle_motor(0);
  
    yaw_motor(true, 255);
    delay(650);

    motors_off();
    delay(355);
    
    measure_position();
    turned = true;
  }
  else {
    // delay(200);
    throttle_motor(0);
    weeee_back(255);
    delay(200);
    weeee_back(0);
    delay(50);

    yaw_motor(false, 255);
    delay(400);
    motors_off();

    throttle_motor(255);
    delay(4000);

    motors_off();
    // delay(355);

    delay(50000);

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
