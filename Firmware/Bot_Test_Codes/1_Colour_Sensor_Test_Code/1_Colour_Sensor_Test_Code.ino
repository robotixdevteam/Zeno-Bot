// Kindly Check whether the Library Files are installed in Library Manager.
#include <Wire.h>
#include <Adafruit_TCS34725.h>
///////////////////color sensor///////////////////
#define TCS3414CS_ADDRESS 0x29 //ColorSensor address 0x29
// Initializing ColorSensor
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X); // Front Color Sensor

void test_CS_Value() {
  uint16_t r, g, b, c;
  tcs1.getRawData(&r, &g, &b, &c);
  uint16_t colorTemp = tcs1.calculateColorTemperature(r, g, b);
  Serial.println("CS Temp Value: "+ String(colorTemp));
}

int get_CS_Value() {

  int col_out;
  const int blue_line_thold = 8000; //Blue Color Temp value
  const int orange_line_thold = 3000; //Orange Color Temp value
  uint16_t r, g, b, c;
  tcs1.getRawData(&r, &g, &b, &c);
  uint16_t colorTemp = tcs1.calculateColorTemperature(r, g, b);

  // WRO Colour Line Condition
  if((0 < colorTemp) && (colorTemp < orange_line_thold) && (colorTemp != 0)) {col_out = 1; } // For Orange Line
  else if((colorTemp > blue_line_thold) && (colorTemp != 0)) {col_out = 3;} // For Blue Line
  else {col_out = 0;}

  return col_out;
}

void setup() {
  Serial.begin(115200);
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  // Set attenuation to 0dB (for full-scale voltage range)       
  analogSetAttenuation(ADC_0db);  
}

void loop() {

  //### Function to test the colour sensor value ###
  test_CS_Value();

  //### Function to test the colour sensor value ###
  // int col = get_CS_Value();
  // Serial.println("CS Color line: "+ String(col));
  
}

