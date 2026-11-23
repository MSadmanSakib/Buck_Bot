// This code is for Lab 1 of ECE 512 2022 Fall Semester
// It was first sketched by Ms. Shalini Manna with debugging and editing input
// from Prof. Mahima Gupta of Portland State University
// Commenting and interpretation and flow has been rearranged by Giri V
// Date of Code is 4 Sept 2022
// Name of file is pindefs.h
// The file is paired with ECE512_Lab_2.ino

// This file lists the pins from Arduino Due that are used in the Octopus Mini (OM) board
// for various housekeeping functions
// Octopus mini was designed by Prof. Mahima Gupta of Portland State Unviersity with
// consulation with Giri V for use with Pego 2.0 power electronic learning system

//Due Board Pin number 49 is for red color led
//Also Port Pin C.14 (or PC14) and Digital pin D49
#define R1 49 

//Due Board Pin number 51 is for green color led
//Also Port Pin C.12 (or PC12) and Digital pin D51
#define G1 51 

//Due Board Pin number 53 is for blue color led
//Also Port Pin B.14 (or PB14) and Digital pin D53
#define B1 53

//Due Board Pin number 23 is for DIP switch 1
//Also Port Pin A.14 (or PA14) and Digital pin D23
#define DIP1 23

//Due Board Pin number 25 is for DIP switch 2
//Also Port Pin D.0 (or PD0) and Digital pin D25
#define DIP2 25

//Due Board Pin number 27 is for DIP switch 3
//Also Port Pin D.2 (or PD2) and Digital pin D27
#define DIP3 27

//Due Board Pin number 29 is for DIP switch 4
//Also Port Pin D.6 (or PD6) and Digital pin D29
#define DIP4 29

//Due Board Pin number 31 is for Toggle switch
//Also Port Pin A.7 (or PA7) and Digital pin D31
#define TOG1 31

//Due Board Pin number 65 is for Potentiometer input
//Also Port Pin B.20 (or PB20) and Analog In A11 (and D65)
#define ADC_11 65

//Due Board Pin number 6 is for Business Monitor output
//Also Port Pin C.24 (or PC24) and Digital pin D6
#define BM 6

//Due Board Pin 33 is for adc_conversion flag called AM
//Also Port Pin C.1 (or PC1) and Digital pin D33
#define AM 33

//Due Board Pin number 9 is for Master Clock
//Also Port Pin C.21 (or PC21) and Digital pin D9

// Due Board Pin number 34 and 35 are for complementary pair of PWML0 and PWMH0
// These are available on PWM Channel 0 of OM labeled PWM0 using USB Type-B connector
// Also Port Pin C.2 (or PC2) and C.3 (or PC3) respectively

// Due Board Pin number 36 and 37 are for complementary pair of PWML1 and PWMH1
// These are available on PWM Channel 1 of OM labeled PWM1 using USB Type-B connector
// Also Port Pin C.4 (or PC4) and C.5 (or PC5) respectively

// Due Board Pin number 38 and 39 are for complementary pair of PWML2 and PWMH2
// These are available on PWM Channel 2 of OM labeled PWM2 using USB Type-B connector
// Also Port Pin C.6 (or PC6) and C.7 (or PC7) respectively

// Due Board Pin number 40 and 41 are for complementary pair of PWML3 and PWMH3
// These are available on PWM Channel 3 of OM labeled PWM3 using USB Type-B connector
// Also Port Pin C.8 (or PC8) and C.9 (or PC9) respectively

// Due Board Pin number 8 and 44 are for complementary pair of PWML5 and PWMH5
// These are available on PWM Channel 5  of OM labeled PWM5 using USB Type-B connector
// Also Port Pin C.22 (or PC22) and C.19 (or PC19) respectively

// Due Board Pin number 7 and 45 are for complementary pair of PWML6 and PWMH6
// These are available on PWM Channel 6  of OM labeled PWM6 using USB Type-B connector
// Also Port Pin C.23 (or PC23) and C.18 (or PC18) respectively

// Chips PS1 and PS2 are connected to PWMH6 and PWML6 respectively for phase-shift functionality.  
// Jumpers J1 and J2 are used to connect PWMH6 and PWML6 respectively to USB Type-B connector PWM6 
// and/or USB Type-B connectors PWM_PS1/PWM_PS2.

// Due Board Pin number 63 (PB18, Analog Pin 9) is for phase-shift chip PS1 for tuning the phase-shift value.
// The output of the PS1 chip is connected to USB Type-B connectors PWM_PS1 and PWM_PS2.

// Due Board Pin number 64 (PB19, Analog Pin 10) is for phase-shift chip PS2 for tuning the phase-shift value.
// The output of the PS2 chip is connected to USB Type-B connectors PWM_PS1 and PWM_PS2.

// Phase-shifted PWMH6 and PWML6 created using PS1 and PS2 connect to USB Type-B connectors PWM_PS1 and PWM_PS2 
// in a complementary manner. 
