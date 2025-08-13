// Kindly Check whether the Library Files are installed in Library Manager.
#include <NewPing.h>

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
#define MAX_DISTANCE 200 //(min, max) = (2, 400) Range

NewPing sonar1(FRONT_TRIGGER, FRONT_ECHO, MAX_DISTANCE); 
NewPing sonar5(FRONT1_TRIGGER, FRONT1_ECHO, MAX_DISTANCE); 
NewPing sonar6(FRONT2_TRIGGER, FRONT2_ECHO, MAX_DISTANCE); 
NewPing sonar2(BACK_TRIGGER, BACK_ECHO, MAX_DISTANCE); 
NewPing sonar3(LEFT_TRIGGER, LEFT_ECHO, MAX_DISTANCE);
NewPing sonar4(RIGHT_TRIGGER, RIGHT_ECHO, MAX_DISTANCE); 

void test_US_Value()
{

  int fus = sonar1.ping_cm();
  int f1us = sonar5.ping_cm();
  int f2us = sonar6.ping_cm();
  int bus = sonar2.ping_cm(); 
  int lus = sonar3.ping_cm(); 
  int rus = sonar4.ping_cm(); 

  Serial.println("F1_US : " + String(f1us) + " | F_US : " + String(fus) + " | F2_US : " + String(f2us) + 
              " | B_US : " + String(bus) + " | L_US : " + String(lus) + " | R_US : " + String(rus));

}

int get_US_Value(String sensorName)
{
  int fus = sonar1.ping_cm();
  int f1us = sonar5.ping_cm();
  int f2us = sonar6.ping_cm();
  int bus = sonar2.ping_cm(); 
  int lus = sonar3.ping_cm(); 
  int rus = sonar4.ping_cm(); 

  if (sensorName == "fus")
    return fus;
  else if (sensorName == "f1us")
    return f1us;
  else if (sensorName == "f2us")
    return f2us;
  else if (sensorName == "bus")
    return bus;
  else if (sensorName == "lus")
    return lus;
  else if (sensorName == "rus")
    return rus;
  else
    return -1; // default case for invalid input
}


void setup() {
  Serial.begin(115200);
}

void loop() {

  //### To test all us values ###
  test_US_Value();

  //### To get each us value ###
  // int fop = get_US_Value("fus");
  // Serial.println("F_US : " + String(get_US_Value("fus")));

}


