// Kindly Check whether the Library Files are installed in Library Manager.
#include <ESP32Servo.h>
#define SERVO_PIN 27 

Servo servo;

// Set the bot servo angles
int center = 100;
int left = center - 20;
int right = center + 20;


void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
}

void set_Servo_To(int angle) {
  angle = constrain(angle, 75, 125); // Constrain the angle between 70 and 130 degrees
  servo.write(angle);               // Move the servo to the specified angle
}

void test_Servo_Motor()
{
  set_Servo_To(left);    // Servo Left turn.
  delay(1000);          // Delay for 1 second.
  set_Servo_To(center);  // Servo Center turn.
  delay(1000);          // Delay for 1 second.
  set_Servo_To(right);   // Servo Right turn.
  delay(1000);          // Delay for 1 second.
  set_Servo_To(center);  // Servo Center turn.
  delay(1000);          // Delay for 1 second.
}

void loop() 
{
  //### Function to test servo motor ###
  test_Servo_Motor();

  //### Function To set Servo Motor at an angle ###
  //set_Servo_To(100);

}