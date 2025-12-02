//                          DEFINITIONS                              //
//*******************************************************************//

  // #define EN_A 6 
  #define IN1 11
  #define IN2 10

  // #define EN_B 5
  #define IN3 9
  #define IN4 6

  // #define Ultrasound sensor pins
  #define TRIGPIN = 12
  #define ECHOPIN = 13

//*******************************************************************//
//                        GLOBAL CONSTANTS                           //
//*******************************************************************//

  // Ultrasound sensor delay limits
  const long max_us = 23000;
  const long min_us = 1000;

  // PID constants
  const float Kp = 0.01; // Proportional gain
  const float Ki = 0.000005; // Integral gain
  const float Kd = 0.000000; // Derivative gain

//*******************************************************************//




void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:

}
