// ########### Loop ############################################################################################################ //
int line_chk_count = 4;  // 3 Laps - 12 count, 1 Lap - 4 count.
int line_count = 0;// 12

int normal_speed = 220;//pwm
int turn_speed = 230;//pwm
int turn_delay = 2000;//ms

const int blue_line_thold = 8000; //Blue Color Temp value
const int orange_line_thold = 3000; //Orange Color Temp value

bool left_right_arc_turn = 1;
bool left_right_r_turn = 0;

int fus_box_dist = 20;
//#---Servo Angles---###################################################################
int servo_center = 100;//deg
int left_turn_angle = servo_center - 20; //80 deg
int right_turn_angle = servo_center + 20;//120 deg
//#########################################################################################//
bool lt_st_count = 0;
bool rt_st_count = 0;
int rbox_count = 0;
int gbox_count = 0;

#define DPDT_Push_Button_Pin 34

int col_val, start_col, colour_line; // colour_line = 1 ->> Orange // colour_line = 3 ->> Blue
int f_us, f1_us, f2_us, b_us, l_us, r_us, fusa, far;

bool COLOR_LOCK = 1;//1 Initial True state.
bool LOGIC_LOCK = 1;//1 Initial True state.
bool DPDT_STATE = 0;//0 Initial False state.
//#########################################################################################//
int animal_detection_value;  // New animal detection value (0-5)
int serial_count;
bool bd_lock = 1;
bool animal_maneuver_in_progress = false; // Flag to prevent interruption during animal maneuvers
bool waiting_for_resume = false; // Flag to indicate waiting for resume signal (8)

const int bufferSize = 1;  
char abd[bufferSize];  

int abdRead;
//#########################################################################################//
// --- Constants for Magic Numbers --- //
const int SUDDEN_BRAKE_DISTANCE_CM = 6 ;
const int SUDDEN_BRAKE_DELAY_MS = 1000;
//#########################################################################################//

void loop() 
{
  
  DPDT_STATE = digitalRead(DPDT_Push_Button_Pin);
  //Serial.println("DPDT Button State : "+String(DPDT_STATE));

  get_US_Values(f_us, f1_us, f2_us, b_us, l_us, r_us);
  //US_Values(f_us, b_us, l_us, r_us);
  // Serial.println("F_US : " + String(f_us) + " | F1_US : " + String(f1_us) + " | F2_US : " + String(f2_us) + 
  //             " | B_US : " + String(b_us) + 
  //             " | L_US : " + String(l_us) + " | R_US : " + String(r_us));

  far = fus_array();
  Serial.println("Front US: " + String(f_us) + " | F1_US: " + String(f1_us) + " | F2_US: " + String(f2_us) + " | far: " + String(far) + " | Waiting for resume: " + String(waiting_for_resume));

  col_val = get_CS_Values();
  //Serial.println("Color Value : " + String(col_val));

  color_line_fun(); // color line pickup

  if (DPDT_STATE == 1) 
  { 
    if (LOGIC_LOCK == 1) 
    { 
      // Check if front obstacle detected (far == 1) and no animal maneuver in progress
      if (far == 1 && !animal_maneuver_in_progress && !waiting_for_resume) {
        Serial.println("Front obstacle detected (far == 1), stopping bot and checking for animal");
        motor_stop(); // Stop the bot immediately
        
        // Now check serial input for animal detection
        animal_detection();
        
        // If no animal detected, keep stopped
        if (animal_detection_value == 0) {
          Serial.println("No animal detected, keeping bot stopped");
        } else {
          Serial.println("Animal detected, waiting for resume signal (8)");
        }
      } else if (animal_maneuver_in_progress) {
        // Animal maneuver is in progress, don't interrupt
        Serial.println("Animal maneuver in progress, ignoring obstacle detection");
      } else if (waiting_for_resume) {
        // Waiting for resume signal (8), keep checking serial
        Serial.println("Waiting for resume signal (8), keeping bot stopped");
        animal_detection();
      } else if (waiting_for_resume) {
        // Still waiting for resume signal, keep stopped
        Serial.println("Still waiting for resume signal (8), keeping bot stopped");
      } else {
        // No front obstacle and not waiting, proceed with normal movement
        Serial.println("No front obstacle, proceeding with normal movement");
        if (col_val != 0)
        {
          color_logic_fun();
        }
        else if (col_val == 0)
        {
          side_us_logic_fun();
        }
      }   

    }
      
  }else{
      bot_shutdown();
  }

  
    

  if (line_count == line_chk_count) 
  {
    end_stop();
    bot_shutdown();
    LOGIC_LOCK = 0;
    line_count = 0; 
  }   

}


void color_line_fun()
{
  if(COLOR_LOCK == 1)
  {
    if(col_val == 1)
    {
      start_col = 1;
      colour_line = 1;
      COLOR_LOCK = 0;
    }
    else if(col_val == 3)
    {
      start_col = 3;
      colour_line = 3;
      COLOR_LOCK = 0;
    }
  }
}

void color_logic_fun()
{

   //# Orange Line & Right Turn Condition: #/////////////////////////////////////////////////////////////////
      if ((col_val == colour_line) && (col_val != 3))
      { 
        line_count++;
        Serial.println("Orange Line Count : " + String(line_count));

        if(left_right_arc_turn)
        {
          rgb_led(0, 0, 0);
          delay(1);  
          motor_forward(turn_speed);
          rgb_led(150, 0, 150);
          moveServoTo(right_turn_angle);
          delay(turn_delay);
          moveServoTo(servo_center);
          delay(1);
          rgb_led(0, 0, 0);
        }

        if(left_right_r_turn)
        {
          rgb_led(0, 0, 0);
          motor_forward(turn_speed);
          rgb_led(150, 0, 150);

          moveServoTo(right_turn_angle);
          delay(1500); 
          motor_backward(turn_speed);
          delay(1);      
          moveServoTo(left_turn_angle);
          delay(1500);
          moveServoTo(servo_center);
          delay(1);  
          motor_forward(210);
          rgb_led(0, 0, 0);
        }
        
      } 
      
      //# Blue Line & Left Turn Condition: #/////////////////////////////////////////////////////////////////
      if ((col_val == colour_line) && (col_val != 1))
      { 
        line_count++;
        Serial.println("Blue Line Count : " + String(line_count));

        if(left_right_arc_turn)
        {
            rgb_led(0, 0, 0);
            rgb_led(150, 0, 150);
            moveServoTo(left_turn_angle);
            delay(turn_delay);
            moveServoTo(servo_center);
            rgb_led(0, 0, 0);
        }


        if(left_right_r_turn)
        {
            rgb_led(0, 0, 0);
            motor_forward(turn_speed);
            rgb_led(150, 0, 150);

            moveServoTo(left_turn_angle);
            delay(1500); 
            motor_backward(turn_speed);
            delay(1);      
            moveServoTo(right_turn_angle);
            delay(1500);
            moveServoTo(servo_center);
            delay(1);  
            motor_forward(210);
            rgb_led(0, 0, 0);

        }
        
      } 

}


void side_us_logic_fun()
{              
    motor_forward(normal_speed);

    if ((l_us < 30) && (l_us != 0))
    { 
        rgb_led(0, 0, 0);
        rgb_led(255, 0, 50); 
        moveServoTo(servo_center + 10);

        if(rt_st_count == 0)
        {
          if(lt_st_count == 0)
          {
            lt_st_count = 1;
          }
        }

    }
    if ((l_us > 30) && (r_us > 30) && (l_us != 0) && (r_us != 0)) 
    { 
        rgb_led(0, 0, 0);
        rgb_led(255, 255, 0);
        moveServoTo(servo_center); //100
    } 

    if ((r_us < 30) && (r_us != 0))
    {
        rgb_led(0, 0, 0);
        rgb_led(255, 0, 50); 
        moveServoTo(servo_center - 10);//90

        if(lt_st_count == 0)
        {
          if(rt_st_count == 0)
          {
            rt_st_count = 1;
          }
        }

    }
}

void bot_shutdown()
{
  motor_stop();
  moveServoTo(servo_center);
  rgb_led(0, 0, 0);
}

void animal_detection()
{
   Serial.println("Checking for serial input... Available: " + String(Serial.available()));
   // Check if data available on serial port
    if( (Serial.available() > 0))  // Only check when far == 1
    { 
      delay(10); // Small delay to ensure complete data reception
      animal_detection_value = Serial.parseInt(); // Read integer value from serial
      serial_count++;
      Serial.println("Animal detected: " + String(animal_detection_value));
    } else {
      Serial.println("No serial data available");
    }
    // else if (far == 1) //animal in front
    // { 
    //   rgb_led(0, 0, 0); 
    //   rgb_led(255, 255, 255); 
    //   motor_stop();
    // }
    // else
    // {
    //   animal_detection_value = 0; // No animal detected
    //   buffer_limit();  //[*]
    // }

    Serial.println("Current animal_detection_value: " + String(animal_detection_value));
    if (animal_detection_value == 1) // && (far == 1)) // Animal Type 1
    {
      Serial.println("Animal Type 1 detected - waiting for camera to finish talking");
      waiting_for_resume = true; // Set flag to wait for resume signal
      rgb_led(0, 0, 0);
      rgb_led(255, 0, 0);
      motor_stop(); // Ensure bot is stopped

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset after handling
      Serial.println("Waiting for resume signal (8) from camera");
    } 

    if(animal_detection_value == 2) // Animal Type 2
    {
      Serial.println("Animal Type 2 detected - waiting for camera to finish talking");
      waiting_for_resume = true; // Set flag to wait for resume signal
      rgb_led(0, 0, 0); 
      rgb_led(0, 255, 0); // Green LED for animal type 2
      gbox_count++;
      motor_stop(); // Ensure bot is stopped

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset after handling
      Serial.println("Waiting for resume signal (8) from camera");
    }

    if(animal_detection_value == 3) // Animal Type 3
    {
      Serial.println("Animal Type 3 detected - waiting for camera to finish talking");
      waiting_for_resume = true; // Set flag to wait for resume signal
      rgb_led(0, 0, 0); 
      rgb_led(0, 0, 255); // Blue LED for animal type 3
      motor_stop(); // Ensure bot is stopped

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset after handling
      Serial.println("Waiting for resume signal (8) from camera");
    }

    if(animal_detection_value == 4) // Animal Type 4
    {
      Serial.println("Animal Type 4 detected - waiting for camera to finish talking");
      waiting_for_resume = true; // Set flag to wait for resume signal
      rgb_led(0, 0, 0); 
      rgb_led(255, 255, 0); // Yellow LED for animal type 4
      motor_stop(); // Ensure bot is stopped

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset after handling
      Serial.println("Waiting for resume signal (8) from camera");
    }

    if(animal_detection_value == 5) // Animal Type 5
    {
      Serial.println("Animal Type 5 detected - waiting for camera to finish talking");
      waiting_for_resume = true; // Set flag to wait for resume signal
      rgb_led(0, 0, 0); 
      rgb_led(255, 0, 255); // Magenta LED for animal type 5
      motor_stop(); // Ensure bot is stopped

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset after handling
      Serial.println("Waiting for resume signal (8) from camera");
    }
    if (animal_detection_value == 8) // Resume forward movement signal from camera
    {
      Serial.println("Resume signal (8) received from camera - resuming normal movement");
      waiting_for_resume = false; // Clear the waiting flag
      rgb_led(0, 0, 0); // Turn off LED
      motor_forward(normal_speed); // Resume normal forward movement
      moveServoTo(servo_center); // Center servo

      buffer_limit(); //[*]
      animal_detection_value = 0; // Reset detection value
      Serial.println("Bot resumed normal movement");
    }
            
}

int fus_array()
{
  if(((f_us <= fus_box_dist) && (f_us != 0)) || 
    ((f1_us <= fus_box_dist) && (f1_us != 0)) || 
    ((f2_us <= fus_box_dist) && (f2_us != 0)) ) { fusa = 1; }

  else{fusa = 0;}

  return fusa;
}

void buffer_limit()
{
  while(Serial.available() > 0)
  {
    // Clear any remaining data in serial buffer
    Serial.read();
  }
}

void left_stop()
{
  rgb_led(0, 0, 0);
  delay(1);
  rgb_led(255, 255, 255);
  
  motor_forward(215);
  delay(1000);
  // moveServoTo(left_turn_angle);
  // delay(1500);
  // moveServoTo(right_turn_angle);
  // delay(1500);
  moveServoTo(servo_center);
  delay(1000);
  
  rgb_led(0, 0, 0);
}

void right_stop()
{
  rgb_led(0, 0, 0);
  delay(1);
  rgb_led(255, 255, 255);

  motor_forward(215);
  delay(1000);
  // moveServoTo(right_turn_angle);
  // delay(1500);
  // moveServoTo(left_turn_angle);
  // delay(1500);
  moveServoTo(servo_center);
  delay(1000);

  rgb_led(0, 0, 0);

}

void end_stop()
{

  if(start_col == 3)
  {
      // Anti-clock wise:
      if(colour_line == 3)
      {
        if(lt_st_count == 1)
        {
          left_stop();
        }
        else if(rt_st_count == 1)
        {
          right_stop();
        }
      }

      if(colour_line == 1)
      {
        if(lt_st_count == 1)
        {
          right_stop();
        }
        else if(rt_st_count == 1)
        {
          left_stop();
        }
      }
  }
  
 ///////////////////////////////////////////////////////////////////
  if(start_col == 1)
  {
      // clock wise:
      if(colour_line == 3)
      {
        if(lt_st_count == 1)
        {
          right_stop();
        }
        else if(rt_st_count == 1)
        {
          left_stop();
        }
      }

      if(colour_line == 1)
      {
        if(lt_st_count == 1)
        {
          left_stop();
        }
        else if(rt_st_count == 1)
        {
          right_stop();
        }
      }
  }

}




// ######## This Firmware was Developed By Meritus AI 2024 ########//
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <FastLED.h>

// ########### Declerations ############################################################################################################ //

// RGB Led

#define LED_PIN 15
#define NUM_LEDS 1

CRGB leds[NUM_LEDS];

// Colour Sensor
#define TCS3414CS_ADDRESS 0x29 //ColorSensor address 0x29

// INTEGRATIONTIME - 2.4 - 614 ms | GAIN - 1x, 4x, 16x, 64x.
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X); // Initializing ColorSensor 

int col_out;

// DC Motor
const int motorPin1 = 32; 
const int motorPin2 = 33; 
const int nslp = 13; 
const int frequency = 5000;

// Servo Motor
#define SERVO_PIN 27
Servo servo;

// Ultrasonic Sensors

#define FRONT_TRIGGER 12 
#define FRONT_ECHO  4  
#define FRONT1_TRIGGER 16
#define FRONT1_ECHO 14
#define FRONT2_TRIGGER 25 
#define FRONT2_ECHO  26  
#define BACK_TRIGGER 17
#define BACK_ECHO 19
#define LEFT_TRIGGER  2
#define LEFT_ECHO  23
#define RIGHT_TRIGGER 5
#define RIGHT_ECHO  18
#define MAX_DISTANCE 400

NewPing sonar1(FRONT_TRIGGER, FRONT_ECHO, MAX_DISTANCE); 
NewPing sonar5(FRONT1_TRIGGER, FRONT1_ECHO, MAX_DISTANCE); 
NewPing sonar6(FRONT2_TRIGGER, FRONT2_ECHO, MAX_DISTANCE); 
NewPing sonar2(BACK_TRIGGER, BACK_ECHO, MAX_DISTANCE); 
NewPing sonar3(LEFT_TRIGGER, LEFT_ECHO, MAX_DISTANCE);
NewPing sonar4(RIGHT_TRIGGER, RIGHT_ECHO, MAX_DISTANCE); 



// ########### Functions ############################################################################################################ //
// RGB Led Function
void rgb_led(int r, int g, int b)
{
  leds[0] = CRGB(r, g, b);
  FastLED.show();
}

// ColorSensor Function

int get_CS_Values() {
  //TCA9548A(0);
  uint16_t r, g, b, c;
  tcs1.getRawData(&r, &g, &b, &c);
  uint16_t colorTemp = tcs1.calculateColorTemperature(r, g, b);
  //Serial.println("CS-1 Color Temp: "+ String(colorTemp));

  // Color Temp Condition
  if((0 < colorTemp) && (colorTemp < orange_line_thold) && (colorTemp != 0))
  {col_out = 1; } 

  else if((colorTemp > blue_line_thold) && (colorTemp != 0))
  {col_out = 3;}
  
  else
  {col_out = 0;}

  return col_out;
}



// DC Motor Functions
void motor_forward(int speed) {
  ledcWrite(5, speed);
  ledcWrite(6, 0);
  //Serial.println("motor_forward");
}

void motor_backward(int speed) {
  ledcWrite(5, 0);
  ledcWrite(6, speed);
  //Serial.println("motor_backward");
}

void motor_stop() {
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  //Serial.println("motor_stop");
}


// Servo Functions
void moveServoTo(int angle) {
  // Constrain the angle between 0 and 180 degrees
  angle = constrain(angle, 75, 125);
  servo.write(angle);
  //delay(15);
  //Serial.println("Servo Angle : "+String(angle));
}


// UltraSonic Function

void get_US_Values(int &f, int &f1, int &f2, int &b, int &l, int &r)
{
  unsigned int front_us = sonar1.ping_cm();
  unsigned int front1_us = sonar5.ping_cm();
  unsigned int front2_us = sonar6.ping_cm();
  unsigned int back_us = sonar2.ping_cm(); 
  unsigned int left_us = sonar3.ping_cm(); 
  unsigned int right_us = sonar4.ping_cm(); 

  f = front_us;
  f1 = front1_us;
  f2 = front2_us;
  b = back_us;
  l = left_us;
  r = right_us;
}

// --- Function to check for sudden brake condition --- //
bool checkFrontObstacle() {
    return ((f_us > 0 && f_us <= SUDDEN_BRAKE_DISTANCE_CM) ||
            (f1_us > 0 && f1_us <= SUDDEN_BRAKE_DISTANCE_CM) ||
            (f2_us > 0 && f2_us <= SUDDEN_BRAKE_DISTANCE_CM));
}

// --- Function to perform sudden brake --- //
void suddenBrake() {
    Serial.println("SUDDEN BRAKE ACTIVATED! Front obstacle detected");
    motor_stop();
    rgb_led(255, 0, 0); // Red LED to indicate emergency stop
    delay(SUDDEN_BRAKE_DELAY_MS);
    Serial.println("Sudden brake completed");
}

// ########### Setup ############################################################################################################ //
void setup() {
  Serial.begin(115200);

  //######### DPDT Setup #########//
  pinMode(DPDT_Push_Button_Pin, INPUT);

  //######### RGB Led Setup #########//
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  //######### Colour Sensor Setup #########//
  analogReadResolution(12);       
  analogSetAttenuation(ADC_0db);
  
  //######### DC Motor Setup ###########//
  ledcSetup(5, frequency, 8);
  ledcSetup(6, frequency, 8);
  ledcAttachPin(motorPin1, 5);
  ledcAttachPin(motorPin2, 6);
  pinMode(nslp, OUTPUT);
  digitalWrite(nslp, HIGH);

  //######### Servo Motor Setup ###########//

  servo.attach(SERVO_PIN, 500, 2400);
  //initial servo angle
  moveServoTo(servo_center);
  delay(500);
  }
