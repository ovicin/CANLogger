
/* SD card */
#include <SD.h>

/* SD card chip select */
const int chipSelect = 10;

void SdCardSetup(){
  ////////////////////////////////////////////////
  /* SD card */
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void SdCardLoop(){
///////////////////////////////////////
    /* data log */
    // make a string for assembling the data to log:
  String dataString = "Bill";

  // read three sensors and append to the string:
  //for (int analogPin = 0; analogPin < 3; analogPin++) {
  //  int sensor = analogRead(analogPin);
  //  dataString += String(sensor);
  //  if (analogPin < 2) {
  //    dataString += ","; 
  //  }
  //}
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
