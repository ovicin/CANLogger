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


/* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
#define PID_0_20            0x00    //PID 0 - 20 supported
#define PID_0_20_DESC               "PID 0x00 - 0x20 Supported"
#define STATUS_DTC          0x01    ///
#define STATUS_DTC_DESC             "Status since DTC Cleared"
#define FREEZE_DTC          0x02    ///
#define FREEZE_DTC_DESC             "Freeze Diagnostic Trouble Code"
#define FUEL_SYS_STATUS     0x03    ///
#define FUEL_SYS_STATUS_DESC        "Fuel System Status"
#define ENGINE_LOAD         0x04    //
#define ENGINE_LOAD_DESC            "Calculated Engine Load"
#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_COOLANT_TEMP_DESC    "Engine Coolant Temperature"
#define ST_FUEL_TRIM_1      0x06    ///
#define ST_FUEL_TRIM_1_DESC         "Short Term Fuel % Trim - Bank 1"
#define LT_FUEL_TRIM_1      0x07    ///
#define LT_FUEL_TRIM_1_DESC         "Long Term Fuel % Trim - Bank 1"
#define ST_FUEL_TRIM_2      0x08    ///
#define ST_FUEL_TRIM_2_DESC         "Short Term Fuel % Trim - Bank 2"
#define LT_FUEL_TRIM_2      0x09    ///
#define LT_FUEL_TRIM_2_DESC         "Long Term Fuel % Trim - Bank 2"
#define FUEL_PRESSURE       0x0A    //
#define FUEL_PRESSURE_DESC          "Fuel Pressure"
#define INTAKE_PRESSURE     0x0B    //
#define INTAKE_PRESSURE_DESC        "Intake Manifold Absolute Pressure"
#define ENGINE_RPM          0x0C
#define ENGINE_RPM_DESC             "Engine RPM"
#define VEHICLE_SPEED       0x0D
#define VEHICLE_SPEED_DESC          "Vehicle Speed"
#define TIMING_ADVANCE      0x0E    //
#define TIMING_ADVANCE_DESC         "Timing Advance"
#define INTAKE_TEMP         0x0F    //
#define INTAKE_TEMP_DESC            "Intake Air Temperature"
#define MAF_SENSOR          0x10
#define MAF_SENSOR_DESC             "MAF Sensor Air Flow Rate"
#define THROTTLE            0x11
#define THROTTLE_DESC               "Throttle Position"
#define COMMANDED_SEC_AIR   0x12    ///
#define COMMANDED_SEC_AIR_DESC      "Commanded Secondary Air Status"
#define O2_SENS_PRES        0x13    ///
#define O2_SENS_PRES_DESC           "Detected O2 Sensors"
#define O2_B1S1_VOLTAGE     0x14    ///
#define O2_B1S1_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 1 Sensor 1"
#define O2_B1S2_VOLTAGE     0x15    ///
#define O2_B1S2_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 1 Sensor 2"
#define O2_B1S3_VOLTAGE     0x16    ///
#define O2_B1S3_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 1 Sensor 3"
#define O2_B1S4_VOLTAGE     0x17    ///
#define O2_B1S4_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 1 Sensor 4"
#define O2_B2S1_VOLTAGE     0x18    ///
#define O2_B2S1_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 2 Sensor 1"
#define O2_B2S2_VOLTAGE     0x19    ///
#define O2_B2S2_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 2 Sensor 2"
#define O2_B2S3_VOLTAGE     0x1A    ///
#define O2_B2S3_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 2 Sensor 3"
#define O2_B2S4_VOLTAGE     0x1B    ///
#define O2_B2S4_VOLTAGE_DESC        "O2 Sensor Voltage - Bank 2 Sensor 4"
#define OBDII_STANDARDS     0x1C    //List of OBDII Standars the car conforms to
#define OBDII_STANDARDS_DESC        "Supported OBDII Standards"
#define O2_SENS_PRES_ALT    0x1D    ///
#define O2_SENS_PRES_ALT_DESC       "Detected O2 Sensors - Alternate Grouping"
#define AUX_IN_STATUS       0x1E    ///
#define AUX_IN_STATUS_DESC          "Auxiliary Input Status"
#define ENGINE_RUNTIME      0x1F    //
#define ENGINE_RUNTIME_DESC         "Run Time Since Engine Started"
#define PID_21_40           0x20    //PID 21-40 supported
#define PID_21_40_DESC              "PID 0x21 - 0x40 Supported"
#define DIST_TRAVELED_MIL   0x21    ///
#define DIST_TRAVELED_MIL_DESC      "Distance Traveled with MIL On"
#define FUEL_RAIL_PRESSURE  0x22    //
#define FUEL_RAIL_PRESSURE_DESC     "Fuel Rail Pressure Relative to Manifold"
#define FUEL_RAIL_PRES_ALT  0x23    ///
#define FUEL_RAIL_PRES_ALT_DESC     "MPI/Diesel Fuel Rail Pressure"
#define O2S1_WR_LAMBDA_V    0x24    ///
#define O2S1_WR_LAMBDA_V_DESC       "O2 Sensor 1 Equivalence Ratio Voltage"
#define O2S2_WR_LAMBDA_V    0x25    ///
#define O2S2_WR_LAMBDA_V_DESC       "O2 Sensor 2 Equivalence Ratio Voltage"
#define O2S3_WR_LAMBDA_V    0x26    ///
#define O2S3_WR_LAMBDA_V_DESC       "O2 Sensor 3 Equivalence Ratio Voltage"
#define O2S4_WR_LAMBDA_V    0x27    ///
#define O2S4_WR_LAMBDA_V_DESC       "O2 Sensor 4 Equivalence Ratio Voltage"
#define O2S5_WR_LAMBDA_V    0x28    ///
#define O2S5_WR_LAMBDA_V_DESC       "O2 Sensor 5 Equivalence Ratio Voltage"
#define O2S6_WR_LAMBDA_V    0x29    ///
#define O2S6_WR_LAMBDA_V_DESC       "O2 Sensor 6 Equivalence Ratio Voltage"
#define O2S7_WR_LAMBDA_V    0x2A    ///
#define O2S7_WR_LAMBDA_V_DESC       "O2 Sensor 7 Equivalence Ratio Voltage"
#define O2S8_WR_LAMBDA_V    0x2B    ///
#define O2S8_WR_LAMBDA_V_DESC       "O2 Sensor 8 Equivalence Ratio Voltage"
#define COMMANDED_EGR       0x2C    //
#define COMMANDED_EGR_DESC          "Commanded EGR"
#define EGR_ERROR           0x2D    //
#define EGR_ERROR_DESC              "EGR Error"
#define COMMANDED_EVAP_P    0x2E    ///
#define COMMANDED_EVAP_P_DESC       "Commanded Evaporative Purge"
#define FUEL_LEVEL          0x2F    //
#define FUEL_LEVEL_DESC             "Fuel Level Input"
#define WARMUPS_SINCE_CLR   0x30    ///
#define WARMUPS_SINCE_CLR_DESC      "Number of Warmups since DTC Cleared"
#define DIST_SINCE_CLR      0x31    ///
#define DIST_SINCE_CLR_DESC         "Distance Traveled Since DTC Cleared"
#define EVAP_PRESSURE       0x32    //
#define EVAP_PRESSURE_DESC          "Evap. System Vapor Pressure"
#define BAROMETRIC_PRESSURE 0x33    //
#define BAROMETRIC_PRESSURE_DESC    "Barometric Pressure"
#define O2S1_WR_LAMBDA_I    0x34    ///
#define O2S1_WR_LAMBDA_I_DESC       "O2 Sensor 1 Equivalence Ratio Current"
#define O2S2_WR_LAMBDA_I    0x35    ///
#define O2S2_WR_LAMBDA_I_DESC       "O2 Sensor 2 Equivalence Ratio Current"
#define O2S3_WR_LAMBDA_I    0x36    ///
#define O2S3_WR_LAMBDA_I_DESC       "O2 Sensor 3 Equivalence Ratio Current"
#define O2S4_WR_LAMBDA_I    0x37    ///
#define O2S4_WR_LAMBDA_I_DESC       "O2 Sensor 4 Equivalence Ratio Current"
#define O2S5_WR_LAMBDA_I    0x38    ///
#define O2S5_WR_LAMBDA_I_DESC       "O2 Sensor 5 Equivalence Ratio Current"
#define O2S6_WR_LAMBDA_I    0x39    ///
#define O2S6_WR_LAMBDA_I_DESC       "O2 Sensor 6 Equivalence Ratio Current"
#define O2S7_WR_LAMBDA_I    0x3A    ///
#define O2S7_WR_LAMBDA_I_DESC       "O2 Sensor 7 Equivalence Ratio Current"
#define O2S8_WR_LAMBDA_I    0x3B    ///
#define O2S8_WR_LAMBDA_I_DESC       "O2 Sensor 8 Equivalence Ratio Current"
#define CAT_TEMP_B1S1       0x3C    ///
#define CAT_TEMP_B1S1_DESC          "Catalyst Temperature Bank 1 Sensor 1"
#define CAT_TEMP_B1S2       0x3E    ///
#define CAT_TEMP_B1S2_DESC          "Catalyst Temperature Bank 1 Sensor 2"
#define CAT_TEMP_B2S1       0x3D    ///
#define CAT_TEMP_B2S1_DESC          "Catalyst Temperature Bank 2 Sensor 1"
#define CAT_TEMP_B2S2       0x3F    ///
#define CAT_TEMP_B2S2_DESC          "Catalyst Temperature Bank 2 Sensor 2"
#define PID_41_60           0x40    //PID 41-60 supported
#define PID_41_60_DESC              "PID 0x41 - 0x60 Supported"
#define MONITOR_STATUS      0x41    ///
#define MONITOR_STATUS_DESC         "Monitor Status This Drive Cycle"
#define ECU_VOLTAGE         0x42    //
#define ECU_VOLTAGE_DESC            "Control Module Voltage"
#define ABSOLUTE_LOAD       0x43    //
#define ABSOLUTE_LOAD_DESC          "Absolute Load Value"
#define COMMANDED_EQUIV_R   0x44    ///
#define COMMANDED_EQUIV_R_DESC      "Commanded Equivalence Ratio"
#define REL_THROTTLE_POS    0x45    ///
#define REL_THROTTLE_POS_DESC       "Relative Throttle Position"
#define AMB_AIR_TEMP        0x46    ///
#define AMB_AIR_TEMP_DESC           "Ambient Air Temperature"
#define ABS_THROTTLE_POS_B  0x47    ///
#define ABS_THROTTLE_POS_B_DESC     "Absolute Throttle Position B"
#define ABS_THROTTLE_POS_C  0x48    ///
#define ABS_THROTTLE_POS_C_DESC     "Absolute Throttle Position C"
#define ACCEL_POS_D         0x49    ///
#define ACCEL_POS_D_DESC            "Accelerator Pedal Position D"
#define ACCEL_POS_E         0x4A    ///
#define ACCEL_POS_E_DESC            "Accelerator Pedal Position E"
#define ACCEL_POS_F         0x4B    ///
#define ACCEL_POS_F_DESC            "Accelerator Pedal Position F"
#define COMMANDED_THROTTLE  0x4C    ///
#define COMMANDED_THROTTLE_DESC     "Commanded Throttle Actuator"
#define TIME_RUN_WITH_MIL   0x4D    ///
#define TIME_RUN_WITH_MIL_DESC      "Time Run with MIL on"
#define TIME_SINCE_CLR      0x4E    ///
#define TIME_SINCE_CLR_DESC         "Time Since DTC Cleared"
#define MAX_R_O2_VI_PRES    0x4F    ///
#define MAX_R_O2_VI_PRES_DESC       "Maximum Value - Equivalence ratio, O2 Voltage, O2 Current, Intake Manifold Pressure"
#define MAX_AIRFLOW_MAF     0x50    ///
#define MAX_AIRFLOW_MAF_DESC        "Maximum MAF Airflow Value"
#define FUEL_TYPE           0x51    //
#define FUEL_TYPE_DESC              "Fuel Type"
#define ETHANOL_PERCENT     0x52    //
#define ETHANOL_PERCENT_DESC        "Ethanol fuel %"
#define ABS_EVAP_SYS_PRES   0x53    ///
#define ABS_EVAP_SYS_PRES_DESC      "absolute Evap. System Vapor Pressure"
#define EVAP_SYS_PRES       0x54    ///
#define EVAP_SYS_PRES_DESC          "Evap. System Vapor Pressure"
#define ST_O2_TRIM_B1B3     0x55    ///
#define ST_O2_TRIM_B1B3_DESC        "Short Term Secondary O2 Sensor Trim - Bank 1 and 3"
#define LT_O2_TRIM_B1B3     0x56    ///
#define LT_O2_TRIM_B1B3_DESC        "Long Term Secondary O2 Sensor Trim - Bank 1 and 3"
#define ST_02_TRIM_B2B4     0x57    ///
#define ST_O2_TRIM_B2B4_DESC        "Short Term Secondary O2 Sensor Trim - Bank 2 and 4"
#define LT_O2_TRIM_B2B4     0x58    ///
#define LT_O2_TRIM_B2B4_DESC        "Long Term Secondary O2 Sensor Trim - Bank 2 and 4"
#define ABS_FUEL_RAIL_PRES  0x59    ///
#define ABS_FUEL_RAIL_PRES_DESC     "Absolute Fuel Rail Pressure"
#define REL_ACCEL_POS       0x5A    ///
#define REL_ACCEL_POS_DESC          "Relative Accelerator Pedal Position"
#define HYBRID_BATT_PCT     0x5B    ///
#define HYBRID_BATT_PCT_DESC        "Hybrid Battery Pack Charge Percent"
#define ENGINE_OIL_TEMP     0x5C    ///
#define ENGINE_OIL_TEMP_DESC        "Engine Oil Temperature"
#define FUEL_TIMING         0x5D    //
#define FUEL_TIMING_DESC            "Fuel Injection Timing"
#define FUEL_RATE           0x5E    //
#define FUEL_RATE_DESC              "Engine Fuel Rate"
#define EMISSIONS_STANDARD  0x5F    ///
#define EMISSIONS_STANDARD_DESC     "Emmissions Requirements"
#define DEMANDED_TORQUE     0x61    ///
#define DEMANDED_TORQUE_DESC        "Driver's Demanded Torque - Percent"
#define ACTUAL_TORQUE       0x62    ///
#define ACTUAL_TORQUE_DESC          "Actual Engine Torque - Percent"
#define REFERENCE_TORQUE    0x63    //
#define REFERENCE_TORQUE_DESC       "Engine Reference Torque"
#define ENGINE_PCT_TORQUE   0x64    ///
#define ENGINE_PCT_TORQUE_DESC      "Engine Percent Torque"
#define AUX_IO_SUPPORTED    0x65    ///
#define AUX_IO_SUPPORTED_DESC       "Auxiliary Input/Output Supported"
#define P_MAF_SENSOR        0x66    ///
#define P_ENGINE_COOLANT_T  0x67    ///
#define P_INTAKE_TEMP       0x68    ///
#define P_COMMANDED_EGR     0x69    ///
#define P_COMMANDED_INTAKE  0x6A    ///
#define P_EGR_TEMP          0x6B    ///
#define P_COMMANDED_THROT   0x6C    ///
#define P_FUEL_PRESSURE     0x6D    ///
#define P_FUEL_INJ_PRES     0x6E    ///
#define P_TURBO_PRESSURE    0x6F    ///
#define P_BOOST_PRES_CONT   0x70    ///
#define P_VGT_CONTROL       0x71    ///
#define P_WASTEGATE_CONT    0x72    ///
#define P_EXHAUST_PRESSURE  0x73    ///
#define P_TURBO_RPM         0x74    ///
#define P_TURBO_TEMP1       0x75    ///
#define P_TURBO_TEMP2       0x76    ///
#define P_CACT              0x77    ///
#define P_EGT_B1            0x78    ///
#define P_EGT_B2            0x79    ///
#define P_DPF1              0x7A    ///
#define P_DPF2              0x7B    ///
#define P_DPF_TEMP          0x7C    ///
#define P_NOX_NTE_STATUS    0x7D    ///
#define P_PM_NTE_STATUS     0x7E    ///
#define P_ENGINE_RUNTUME    0x7F    ///
#define P_ENGINE_AECD_1     0x81    ///
#define P_ENGINE_AECD_2     0x82    ///
#define P_NOX_SENSOR        0x83    ///
#define P_MANIFOLD_TEMP     0x84    ///
#define P_NOX_SYSTEM        0x85    ///
#define P_PM_SENSOR         0x86    ///
#define P_IN_MANIF_TEMP     0x87    ///
 
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


//This code needs to be adapted in order to compile in my implementation

/*
typedef struct
{
	uint16_t id;
	struct {
		int8_t rtr : 1;
		uint8_t length : 4;
	} header;
	uint8_t data[8];
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
	

	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//		SET(LED2_HIGH);	
	if (mcp2515_send_message(&message)) {
	}
	
	while(timeout < 4000)
	{
		timeout++;
				if (mcp2515_check_message()) 
				{

					if (mcp2515_get_message(&message)) 
					{
							if((message.id == PID_REPLY) && (message.data[2] == pid))	// Check message is the reply and its the right PID
							{
								switch(message.data[2])
								{   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
									case ENGINE_RPM:  			//   ((A*256)+B)/4    [RPM]
									engine_data =  ((message.data[3]*256) + message.data[4])/4;
									sprintf(buffer,"%d rpm ",(int) engine_data);
									break;
							
									case ENGINE_COOLANT_TEMP: 	// 	A-40			  [degree C]
									engine_data =  message.data[3] - 40;
									sprintf(buffer,"%d degC",(int) engine_data);
							
									break;
							
									case VEHICLE_SPEED: 		// A				  [km]
									engine_data =  message.data[3];
									sprintf(buffer,"%d km ",(int) engine_data);
							
									break;

									case MAF_SENSOR:   			// ((256*A)+B) / 100  [g/s]
									engine_data =  ((message.data[3]*256) + message.data[4])/100;
									sprintf(buffer,"%d g/s",(int) engine_data);
							
									break;

									case O2_VOLTAGE:    		// A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
									engine_data = message.data[3]*0.005;
									sprintf(buffer,"%d v",(int) engine_data);
							
									case THROTTLE:				// Throttle Position
									engine_data = (message.data[3]*100)/255;
									sprintf(buffer,"%d %% ",(int) engine_data);
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
 */

