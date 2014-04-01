/*********************

Arduino Uno + Adafruit LCD shield + SparkFun LCD Shield + SDCard 

Dependencies:
  Adafruit LCD library
    https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library/
  CAN Shield
    currently using:   http://www.seeedstudio.com/wiki/images/0/0c/CAN_BUS_Shield_Code.zip
    tested in the car: https://github.com/yexiaobo-seeedstudio/CAN_BUS_Shield
    https://github.com/Seeed-Studio/CAN_BUS_Shield

**********************/

// include the library code:
#include <Wire.h>

void setup() {
  // Debugging output
  Serial.begin(115200);
  
  //LcdSetup();
  //SdCardSetup();
  CANSetup();
}



//uint8_t i=0;
void loop() {
  //LcdLoop();
  CANLoop();
  //SdCardLoop();  
}
