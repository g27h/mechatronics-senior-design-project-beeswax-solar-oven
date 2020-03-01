// Author: Gustavo Hurtado
// Purpose: To drive the Solar Beeswax Oven
// Methodology for the Code: 
//  1. Receive both signals from the solar photocells and use an ADC on the Arduino to turn them into digital signals
//  2. Use a simple differential variable to hold the difference between the two photocell values
//  3. Have a PI controller act upon the difference on the photocell values
//    3a. 
//  4. Have a PI controller for the Temperature setpoint to change the Voltage setpoint for the other PI controller
// Methodology for the Hardware:
//  1. Have capacitors on the photocells to act as a low-pass filter

//State Variables
static volatile bool SunTrackingMode; //Mode for purely tracking the sun
static volatile bool TempControllingMode; //Mode for adding the additional functionality of the temperature controlling
static volatile bool ReturnToHome; //Mode for returning to home when either the day ends, or the bed is full, this can act on a timer or on the magnitude of the voltages
                                  //coming into the sensors

//Numerical Variables - Photocells
static volatile uint16_t V_cell_1; //The voltage from photocell 1
static volatile uint16_t V_cell_2; //The voltage from photocell 2
static volatile uint16_t V_diff; //The difference in voltage of the photocells
static volatile uint8_t V_setpoint;

//Numerical Variables - Temperature
static volatile uint16_t T_bed; //The temperature in the bed
static volatile uint16_t T_outside; //The outside temperature
static volatile uint8_t T_setpoint;
static volatile uint16_t W_bed; //The weight of the bed


void setup() {

}

void loop() {

}
