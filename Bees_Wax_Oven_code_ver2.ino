// Author: Gustavo Hurtado
// Purpose: To drive the Solar Beeswax Oven
// Methodology for the Code: 
//  1. Receive both signals from the solar photocells and use an ADC on the Arduino to turn them into digital signals
//  2. Use a simple differential variable to hold the difference between the two photocell values
//  3. Have a P controller act upon the difference on the photocell values
//    3a. 
//  4. Have a P controller for the Temperature setpoint to change the Voltage setpoint for the other PI controller
// Methodology for the Hardware:
//  1. Have capacitors on the photocells to act as a low-pass filter



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
  static volatile uint16_t V_setpoint_mod; //setpoint modifier
  
  //Numerical Variables - Temperature
  static volatile uint16_t T_bed; //The temperature in the bed
  static volatile uint16_t T_outside; //The outside temperature
  static volatile uint16_t T_setpoint_high; //The desired max temperature for the wax to operate at
  static volatile uint16_t T_setpoint_low; //The desired max temperature for the wax to operate at
  static volatile uint16_t W_bed; //The weight of the bed

  //Numerical Variables - Errors
  static volatile uint16_t V_error;
  static volatile uint16_t T_error;

  //PWM / Motor Variables
  static volatile uint16_t PWM;
  static volatile bool driveDirection;
}

void setup() {
  
  VariableSetup();
  
  SunTrackingMode = true;
  TempControllingMode = false;
  ReturnToHome = false;

  V_setpoint = 0; //0V setpoint in the beginning
  T_setpoint = 65;

  k_p_V = 50;
  k_p_T = 10;
  
}

void loop() {
  //Read left voltage
  //Read right voltage

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
    //Runs the motor to the home position
  }
}

//Created to 
void sunTrackingModeFunct(){
  PWM = abs(k_p_V*V_error);
  if(PWM > 99){
    PWM = 100;
  }
  if(V_error < 0){ //more sunlight on the left cell
    //drive the motor to move the oven CW
    driveMotor(true,PWM);
  }
  else{ //more sunlight on the right cell
    //drive the motor to move the oven CCW
    driveMotor(false,PWM);
    
  }
}

void driveMotor(bool driveDirection, uint16_t PWM){
  if(driveDirection = true){
    //drive the motor CW
  }
  else{ //direction = false
    //drive the motor CCW
  }
}

