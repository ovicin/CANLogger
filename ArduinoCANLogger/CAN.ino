

////////////////////////////////
/* CAN shield */
unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];


#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8



/*
1. Initialize CAN module for 500kbps.

2. Use standard (11-bit) ID settings. No need of extended  (29-bit) ID.

3. Set receiver filter to 0x7E8.

4. Enable receive interrupt.

5. Initialize LCD module.

6. Set the transmit CAN message packet as below: -

a.                       canTxMessage.id=0x7DF;

b.                      canTxMessage.data[0]=0x02;

c.                       canTxMessage.data[1]=0x01;

d.                      canTxMessage.data[2]=ENGINE_SPEED_PID;

e.                      canTxMessage.data[3]=0x00;

f.                        canTxMessage.data[4]=0x00;

g.                       canTxMessage.data[5]=0x00;

h.                      canTxMessage.data[6]=0x00;

i.                         canTxMessage.data[7]=0x00;

j.                        canTxMessage.data_length=8;

7. Transmit the CAN packet.

8. Inside “while” loop, wait for CAN message to be received.

9. If the CAN message is received, it will be for sure from the ECU (because, we set the filter accordingly), check the message byte – 2. Accordingly, convert the byte -3 and 4 into human readable message format and display over LCD
*/



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
  switch (MainMenu){
  //case SNIFFER:
      CAN_sniffer();
  //  break;
  //case OBD:
      ecu_req(ENGINE_RPM,  engineRPM);
  //  break;
  }
}


void CAN_sniffer(void){
char printBuff[8];
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
            printBuff[i] = buf[i];
          } //PRINT
          CAN_LOG(millis(), idrecieved, "", printBuff);
        }
      }
    }
}


typedef struct
{
	INT32U id;
	struct {
		INT8U rtr : 1;
		INT8U length : 4;
	} header;
	INT8U data[8];
} tCAN;

char ecu_req(unsigned char pid,  char *buffer) 
{
	tCAN message;
	float engine_data;
	int timeout = 0;
	char message_ok = 0;
	// Prepair message
	message.id = PID_REQUEST;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x02;
	message.data[1] = 0x01;
	message.data[2] = pid;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	INT8U len=8;

//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//		SET(LED2_HIGH);	
//	if (mcp2515_send_message(&message)) {
//	}

	if(CAN.sendMsgBuf(message.id, 0, message.header.length, message.data)){
        }  

	while(timeout < 4000)
	{
		timeout++;
				//if (mcp2515_check_message()) 
                                if (CAN.checkReceive()) 
				{

					//if (mcp2515_get_message(&message)) 
                                        if(CAN.readMsgBuf(&len,message.data))
					{
                                                        message.id = CAN.getCanId();
							if((message.id == PID_REPLY) && (message.data[2] == pid))	// Check message is the reply and its the right PID
							{
								switch(message.data[2])
								{   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
									case ENGINE_RPM:  			//   ((A*256)+B)/4    [RPM]
									engine_data =  ((message.data[3]*256) + message.data[4])/4;
									sprintf(buffer,"%d rpm ",(int) engine_data);
                                                                        CAN_LOG(millis(),message.id,"ENGINE_RPM",buffer);
									break;
							
									case ENGINE_COOLANT_TEMP: 	// 	A-40			  [degree C]
									engine_data =  message.data[3] - 40;
									sprintf(buffer,"%d degC",(int) engine_data);
							                CAN_LOG(millis(),message.id,"ENGINE_COOLANT_TEMP",buffer);
									break;
							
									case VEHICLE_SPEED: 		// A				  [km]
									engine_data =  message.data[3];
									sprintf(buffer,"%d km ",(int) engine_data);
							                CAN_LOG(millis(),message.id,"VEHICLE_SPEED",buffer);
									break;

									case MAF_SENSOR:   			// ((256*A)+B) / 100  [g/s]
									engine_data =  ((message.data[3]*256) + message.data[4])/100;
									sprintf(buffer,"%d g/s",(int) engine_data);
							                CAN_LOG(millis(),message.id,"MAF_SENSOR",buffer);
									break;

									/*case O2_VOLTAGE:    		// A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
									engine_data = message.data[3]*0.005;
									sprintf(buffer,"%d v",(int) engine_data);
							*/
									case THROTTLE:				// Throttle Position
									engine_data = (message.data[3]*100)/255;
									sprintf(buffer,"%d %% ",(int) engine_data);
                                                                        CAN_LOG(millis(),message.id,"THROTTLE",buffer);
									break;
							
								}
								message_ok = 1;
							}

					}
				}
				if(message_ok == 1) return 1;
	}


 	return 0;
}
 

