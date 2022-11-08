#include "TeensyStep.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce.h>

Encoder knob(11, 12);
const int encoder_SW = 13;
Bounce pushbutton = Bounce(encoder_SW, 10);  // 10 ms debounce
bool state = 0;

Stepper pump(3, 4);       // STEP pin: 7, DIR pin: 6  // The stepper class encapsulates the physical properties of a stepper motor like pin numbers of the STEP and DIR signals, speed and acceleration of the motor.
Stepper pump2(6, 7); 

//StepControl step_controller;    // The StepControl class is used to synchronously move up to 10 motors to their target positions.
RotateControl rotate_controller; // The RotateControl class is used to synchronously rotate up to 10 motors.
RotateControl rotate_controller2; // The RotateControl class is used to synchronously rotate up to 10 motors.
int speed_ = 0;

//Fluid global constants
const int lead_size = 8; // lead size (8mm)
//const int lead_size = 1; // lead size (1mm)
const float steps_rev = 3200; // Stepper resolution at 1/16 steps per revolution
const float syringe_area = 175; 
float linear_resolution = (lead_size/steps_rev)*syringe_area; //uL/step or mm^3/step

float Volume;
float flow_rate;
int steps;
int steps_s;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MS1 = 8;
const int MS2 = 9;
const int MS3 = 10;
const int Enable = 2;
const int Enable2 = 5;

//Direction
bool dir = false;

void setup()
{
  pinMode(MS1, OUTPUT);    // set the MS1, MS2, MS3 and Enable as an outputs
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(Enable, OUTPUT);

  digitalWrite(MS1, HIGH); // sets the digital pin MS1, MS2, MS3 to HIGH
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  //digitalWrite(Enable, HIGH); // set the enable to HIGH
  //digitalWrite(Enable2, HIGH); // set the enable to HIGH
  
  // Set the motor max acceleration
  pump.setAcceleration(500)   // stp/s^2
      .setInverseRotation(true);

  pump2.setAcceleration(500)   // stp/s^2
      .setInverseRotation(true);

  pinMode(encoder_SW, INPUT_PULLUP);
  
  Serial.begin(115200);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000); 
  display.clearDisplay();

  digitalWrite(Enable, LOW);
  digitalWrite(Enable2, LOW);

}

void loop() 
{ 
//  speed_ = abs(knob.read()/2);
//
//  //Serial.println(speed_);
//  pump.setMaxSpeed(speed_);
//  pump2.setMaxSpeed(speed_);
//
//  //Serial.println(speed_);
//  rotate_controller.overrideSpeed(speed_);
//  rotate_controller2.overrideSpeed(speed_);
//  
//  //rotate_controller.rotateAsync(pump, pump_2, pump_3, pump_4);
//  rotate_controller.rotateAsync(pump);
//  rotate_controller2.rotateAsync(pump2);
//
//  OLED_display();
  
  read_pushbutton();
  
  speed_ = abs(knob.read()/2);

  OLED_display();
 
  digitalWrite(Enable, HIGH);
  digitalWrite(Enable2, HIGH);
  
  rotate_controller.stop();
  rotate_controller2.stop();
  
  while (state == HIGH)
  {
    read_pushbutton();
        
    digitalWrite(Enable, LOW);
    digitalWrite(Enable2, LOW);
    speed_ = abs(knob.read()/2);
  
    //Serial.println(speed_);
    pump.setMaxSpeed(speed_);
    pump2.setMaxSpeed(speed_);
    
    //rotate_controller.rotateAsync(pump, pump_2, pump_3, pump_4);
    rotate_controller.rotateAsync(pump);
    rotate_controller2.rotateAsync(pump2);
  
    OLED_display();
 }
  

}

int get_speed(float flow_rate)
{
  float uL_s = flow_rate/60;
  int steps_s;
  steps_s = uL_s*(1/linear_resolution);
  return steps_s;
}

int get_steps(int Volume)
{
  int linear_displacement = Volume/linear_resolution;
  return linear_displacement;
}

float get_volume(int steps)
{
  int Volume = steps*linear_resolution;
  return Volume;
}

void OLED_display()
{   
    display.clearDisplay();
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print(F("Perfusion syste,"));
    display.setTextSize(1);
    display.println();
    display.println();
    display.print(F("Flow rate selected:"));
    display.println();
    display.setTextSize(1);
    display.print(knob.read());
    display.setTextSize(1);
    display.println  (F(" steps"));
//    display.print(F("Flow rate dispensed:"));
//    display.println();
//    display.print(rotate_controller.getCurrentSpeed()*linear_resolution*60); 
//    display.println  (F(" uL/min"));
    display.print(F("Controller 1:"));
    display.println();
    display.print(rotate_controller.getCurrentSpeed()); 
    display.println  (F(" Steps"));
    display.setTextSize(1);
    display.print(F("Controller 2:"));
    display.println();
    display.print(rotate_controller2.getCurrentSpeed()); 
    display.println  (F(" Steps"));
//    display.print(F("Volume:"));
//    display.print(int(Volume));
//    display.setTextSize(1);
//    display.println(F(" uL"));
    display.display(); 
}

void read_pushbutton()
{
    if (pushbutton.update()) 
    {
      if (pushbutton.fallingEdge()) 
      {
        state = !state;
      }
    } 
}
