///////////////////////////////
/* CAN Shield*/
#include <mcp_can.h>
#include <SPI.h>

////////////////////////////////
/* CAN shield */
unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];


#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE			0x11

#define PID_REQUEST         0x7DF
#define PID_REPLY			0x7E8

void CANSetup(){

 /////////////////////////////////////
  /* CAN shield setup */
  CAN.begin(CAN_500KBPS);                       // init can bus : baudrate = 500k
  attachInterrupt(0, MCP2515_ISR, FALLING);     // start interrupt
}

void MCP2515_ISR()
{
    Flag_Recv = 1;
}

void CANLoop(){
//////////////////////////
  /* CAN shield */
   if(Flag_Recv)                           // check if get data
    {
      Flag_Recv = 0;                        // clear flag
      CAN.readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
      Serial.println("CAN_BUS GET DATA!");
      Serial.print("data len = ");
      Serial.println(len);
      
      for(int i = 0; i<len; i++)            // print the data
      {
        Serial.print(buf[i]);Serial.print("\t");
      }
      Serial.println();
    }
    
    /* */
    if(CAN.checkReceive())
    {
      if (CAN.checkError()){
        Serial.println("CAN Error");
      }
      else{
        int idrecieved = CAN.getCanId();
        if (idrecieved){
          Serial.print("DATA idrecieved =");
          Serial.println(idrecieved);
          //READ
          CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
          Serial.print("DATA idrecieved len = ");Serial.println(len);
          for(int i = 0; i<len; i++)    // print the data
          {
            Serial.print(buf[i]);Serial.print("\t");
          } //PRINT
        }
      }
    }
}
