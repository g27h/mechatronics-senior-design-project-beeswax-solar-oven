// Author: Gustavo Hurtado
// Purpose: To drive the Solar Beeswax Oven
// Methodology for the Code: 
//  1. Receive both signals from the solar photocells and use an ADC on the Arduino to turn them into digital signals
//  2. Use a simple differential variable to hold the difference between the two photocell values
//  3. Have a P controller act upon the difference on the photocell values while in the "Sun Tracking Mode"
//    3a. Calculates the PWM value based on the k_p_V constant and voltage error from the setpoint
//  4. Have a P controller for the Temperature setpoint to change the Voltage setpoint for the other P controller
//    4a. Calculates the new setpoint value for the other P controller based on the k_p_T constant, the Temperature error and the voltage ratio
//    4b. The voltage ratio is used to determine the sign of the setpoint and a small fraction of the magnitude
//  5. TODO: Finally the code features a "Return to Home" mode which can be activated via an interrupt from a button or strictly from the day ending
//    5a. TODO: The method of having the solar oven return to home is somewhat unclear, and may deal with the voltages reaching a low value
//    5b. TODO: May need an encoder and two buttons to help set the angular limits of the solar oven
// Methodology for the Hardware:
//  1. Have capacitors on the photocells to act as a low-pass filter
//  2. Use a Qunqi L298N motor controller board (which has the capacity of up to 2 motors)
//    2a. The motor controller board is capable of running up to 12V and has a regulator
//      2a1. The motor runs up to 12V (though we will not be running strictly at that voltage ever)
//      2a2. The 5V supply can be used to power the arduino itself


void VariableSetup() {

  //State Variables
  static volatile bool sunTrackingMode; //Mode for purely tracking the sun
  static volatile bool tempControllingMode; //Mode for adding the additional functionality of the temperature controlling
  static volatile bool returnToHome; //Mode for returning to home when either the day ends, or the bed is full, this can act on a timer or on the magnitude of the voltages
                                    //coming into the sensors
  //Proportional Constants
  static volatile uint16_t k_p_V; //proportional constant for the photocells
  static volatile uint16_t k_p_T; //proportional constant for the temperature
  
  //Numerical Variables - Photocells
  static volatile uint16_t V_cell_L; //The voltage from photocell 1 (left)
  static volatile uint16_t V_cell_R; //The voltage from photocell 2 (right)
  static volatile uint16_t V_diff; //The difference in voltage of the photocells
  static volatile uint16_t V_setpoint; //The desired voltage difference setpoint. Initially this is zero which is unity between the two photocells
  
  //Numerical Variables - Temperature
  static volatile uint16_t T_bed; //The temperature in the bed
  static volatile uint16_t T_outside; //The outside temperature
  static volatile uint16_t W_bed; //The weight of the bed

  //Numerical Variables - Errors
  static volatile uint16_t V_error;
  static volatile uint16_t T_error;

  //Motor Variables
  static volatile uint16_t PWM;
  static volatile bool driveDirection;
  static volatile int16_t enA;
  static volatile int16_t in1;
  static volatile int16_t in2;
}

void setup() {
  
  VariableSetup();
  
  SunTrackingMode = true;
  TempControllingMode = false;
  ReturnToHome = false;

  V_setpoint = 0; //0V setpoint in the beginning
  T_setpoint = 65; //65 deg F setpoint

  k_p_V = 50; //arbitrary kp constant
  k_p_T = 10; //arbitrary kp constant

  // Motor variables
  // The motor is controlled via a HIGH to IN1 and a LOW to IN2 to turn in one dir
  // or a LOW to IN1 and a HIGH to IN2. In the cases of sending the high signal
  // we instead use PWM
  enA = 10; //PWM/analog pin 10, connected to module pin 7 (remove the jumper first)
  in1 = 9; //digital pin 9, connected to IN1
  in2 = 8; //digital pin 8, connected to IN2

  //Setting pins 10, 9 and 8 as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  //TODO: Read left voltage and store it using the ADC
  //TODO: Read right voltage and store it using the ADC

  V_diff = V_cell_L - V_cell_R; //left - right sensor voltages, thus this will typically be positive
  
  //positive error -> right sensor is higher, and need to move the oven CCW, (closer to the home position)
  //negative error -> left sensor is higher, and need to move the oven CW 
  V_error = V_setpoint - V_diff; //typically will be negative (since the setpoint starts at zero.)
  T_error = T_setpoint - T_bed;
  
  if(T_error < 0){ //i.e. the bed temp is greater than the setpoint 
    tempControllingMode = true;
    sunTrackingMode = false;
  }
  
  if(sunTrackingMode){ 
    sunTrackingModeFunct();
  }
  // want the temperature controlling mode to lower the temperature by adjusting the voltage setpoint
  else if(tempControllingMode){
    //if the ratio between the voltages is unity, then the setpoint doesn't change
    //if the ratio is above 1, then the left sensor has more sunlight, and the setpoint is driven to be more positive
    //if the ratio is below 1, then the right sensor has more sunlight, and the setpoint is driven to be more negative
    V_setpoint = k_p_T*T_error*(1 - V_cell_R/V_cell_L);
    sunTrackingModeFunct();
  }
  //May need an encoder for the return home variable and a push button to home the oven
  else if(returnHome){
    //TODO: Runs the motor to the home position
  }
}

//Created to be a standalone function for tracking the sun. Will be used in the 
//temperature controlling mode as well.
void sunTrackingModeFunct(){
  PWM = abs(k_p_V*V_error);
  if(PWM > 255){ //the range on an arduino is 0~255
    PWM = 255;
  }
  if(V_error < 0){ //more sunlight on the left cell
    //drive the motor to move the oven CW
    driveMotor(true, PWM);
  }
  else{ //more sunlight on the right cell
    //drive the motor to move the oven CCW
    driveMotor(false, PWM);
  }
}

void driveMotor(bool driveDirection, uint16_t PWM){
  if(driveDirection = true){
    //drive the motor CW using a PWM pin, also a motor control direction pin
    //TODO: guessing here on the drive direction for the in1 and in2..
    analogWrite(enA, PWM);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{ //direction = false
    //drive the motor CCW using a PWM pin, also a motor control direction pin
    //TODO: guessing here on the drive direction for the in1 and in2..
    analogWrite(enA, PWM);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}
