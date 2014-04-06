
/* SD card */
#include <SD.h>

/* SD card chip select */
const int chipSelect = 10;

File dataFile;

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

}

void CAN_LOG(unsigned long time, INT32U id, String Name, char *buf){
  String LogString, DataString;
  LogString = "";
  DataString = "";
  for (int i=0; i < 8; i++){
    DataString += ", ";
     DataString += String(buf[i]);
  }
  LogString = String(time) + ", " + String(id) + ", " + Name + DataString;
  
  LOG(DataString);
} 

void LOG(String dataString){
  dataFile = SD.open("datalog.txt", FILE_WRITE);
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
