
#include "HX711.h"

#define calibration_factor -13350.0 //This value is obtained using the SparkFun_HX711_Calibration sketch
#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN  2
float curren=0;
float previo=0;
float scaled=0;
float max_val=0;
HX711 scale;

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 scale demo");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0

  Serial.println("Readings:");
}

void loop() {
  curren=scale.get_units()*1000;
  scaled=0.8*curren + 0.2*previo;
  previo=scaled;
  Serial.print(-scaled); //scale.get_units() returns a float
  previo=-curren;
  //Serial.print("g"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();
}
