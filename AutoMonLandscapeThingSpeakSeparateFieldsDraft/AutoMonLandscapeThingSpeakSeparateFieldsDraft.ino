//AutoMonLandscapeThingSpeakSeparateFieldsDraft
//Date Modified & Created: 8 April 2024 From Sketch REMETAutoMonLandscapeThingSpeakVer1Fixes
//Dates Revised: April 18, 2024
//Goals: Separate the data fields and graphs for battery level and node info (8April2024); Have a decimal place reflected in ThingSpeak for the battery level & improve its reading accuracy
//Progress Made: Able to create new separate variable for the battery level data (8April2024); Able to separate & print three data; able to separate the fields for water & battery level(12April2024); 
// able to have three separate data fields for water level, battery level & node ID but the battery level decimal place did not appear; able to display graphs for water & battery level (15April2024); 
//Able to change the buf variable for node ID to buf3
//Limitations: The battery level data has no decimal place yet in ThingSpeak and there's reading battery issue (15April2024)
//Modified and Developed by: Stephen Brix L. Racal, Asst. Scientist - Electronics Engineering, Computer Science & Instrumentation
//Original Sketch Developed by Freaklabs & IRRI, Philippines


#include "EasyLink.h"
#include <CmdGateway.h>
#include <stdio.h>
#include <string.h> //16April2024
#include <stdlib.h> //15April2024 needed for atoi & atof functions
#include "float.h" //15April2024
#include <math.h>  //15April2024
#include <uart.h>
#include <limits.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/boards/CC1310_LAUNCHXL/CC1310_LAUNCHXL.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <SC16IS750.h>
#include <rtc3231.h>
#include <IrriGSM.h>
#include <driverlib/osc.h>
#include "ioc.h"
#include "prcm.h"


// set to 1 if you are testing and don't want to send an SMS
#define TESTMODE 0

// enable duplicate checking. For testing, sometimes more convenient to disable
#define DUPECHECK_ENABLE 0

/******************************************************/
// only one of these should be set to 1
#define BUILD_LONGTERM      0
#define BUILD_PHILRICE      0
#define BUILD_LABTEST       1

// select defines based on build
//#define LONGTERM_PHONE  "+639958747117"  // IRRI1
#define LONGTERM_PHONE   "+639561823955"     // IRRI2
#define LONGTERM_GWADDR 0xaa

//#define LABTEST_PHONE   "+639984366562"
//#define LABTEST_PHONE   "+639958747142"
#define LABTEST_PHONE   "+639603510337"
#define LABTEST_PHONE2  "+639603510337"
#define LABTEST_GWADDR  0xbb

//#define PHILRICE_PHONE  "+639614709715"
#define PHILRICE_PHONE  "+639614709715"
#define PHILRICE_PHONE2  "+639603510337"
#define PHILRICE_GWADDR 0xcc
/******************************************************/

#if (BUILD_LONGTERM == 1)
    #define SMS_PHONE   LONGTERM_PHONE
    #define GW_ADDR     LONGTERM_GWADDR 
#elif (BUILD_PHILRICE == 1)
    #define SMS_PHONE   PHILRICE_PHONE
    #define SMS_PHONE2   PHILRICE_PHONE2    
    #define GW_ADDR     PHILRICE_GWADDR
#elif (BUILD_LABTEST == 1)
    #define SMS_PHONE   LABTEST_PHONE
    #define SMS_PHONE2  LABTEST_PHONE2  
    #define GW_ADDR     LABTEST_GWADDR
#else
    #error "NO ACCEPTABLE BUILD DEFINITION SPECIFIED"
#endif

// general timeouts
// watchdog timeout needs to be longer than the receive timeout
// or else watchdog will reset system
#define RX_TIMEOUT_MS       60000 
#define WD_TIMEOUT_MS       RX_TIMEOUT_MS + 10000

// radio defines
#define Board_HGM       CC1310_LAUNCHXL_DIO28_ANALOG
#define Board_LNA_EN    CC1310_LAUNCHXL_DIO29_ANALOG
#define Board_PA_EN     CC1310_LAUNCHXL_DIO30_ANALOG

// general defines
#define BUF_SZ                  200
#define FUNCTIONALMODE          "T"
#define DEST_NODE_ADDR          0x55
#define DUPETABLE_MAX_ENTRIES   5
#define PACKET_MAXSIZE          100
#define NUMPACKETS              2 // changed from 2 to 1 if want to have one Node only to transmit SMS at a time(13March2024);  better 2 for GPRS (19March2024)


// easylink params
EasyLink_RxPacket rxPacket;
EasyLink_TxPacket txPacket;
EasyLink myLink;

typedef struct
{
    uint8_t     addr[8];
    uint32_t    seq;
} dupe_t;

typedef struct
{
    uint8_t     addr[8];
    int16_t    dist[3];
    float       batt;
    float       temperature;
    int8_t      rssi;
    uint8_t     retries;
    ts_t        timestamp;
} pktData_t;

// set up SPI UART
uint8_t pinUartCsN = 18;    // DIO11
uint8_t pinLed0 = 39;       // DIO6
uint8_t pinLed1 = 40;       // DIO7
uint8_t pinHgm = 26;

SC16IS750 spiuart = SC16IS750(SC16IS750_PROTOCOL_SPI, pinUartCsN); 

// set up SIM800L modem
IrriGSM gsm = IrriGSM();

PIN_Handle rfx1010PinHandle;
static PIN_State rfx1010PinState;
PIN_Config rfx1010PinTable[] =
{
    Board_HGM | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LNA_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PA_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE,
};

dupe_t dupeTable[DUPETABLE_MAX_ENTRIES];
pktData_t packet[NUMPACKETS];

// watchdog params
Watchdog_Handle watchdogHandle;
Watchdog_Params params;
uint32_t reloadValue;
uint8_t thisAddr[8];
char buf[BUF_SZ];
char buf1[BUF_SZ]; //11April2024
char buf2[BUF_SZ];//8April2024
char buf3[BUF_SZ];//18April2024
uint8_t pktIndex = 0;


// this is the only address that we will receive from
uint8_t gatewayAddrFilter[1] = {GW_ADDR}; 

int dummyData = 101;
char BatteryLevel;//8April2024
//int x = 4.2;//11April2024 for bus asgmt

/**************************************************************************/
// SETUP
/**************************************************************************/
void setup()
{ 
    // set up easylink-/
    rfx1010PinHandle = PIN_open(&rfx1010PinState, rfx1010PinTable);
    
    // begin defaults to EasyLink_Phy_50kbps2gfsk
    myLink.begin(EasyLink_Phy_625bpsLrm);
//    myLink.begin();

    // reset dupeTable
    memset(dupeTable, 0, sizeof(dupeTable));
    
    PINCC26XX_setMux(rfx1010PinHandle, Board_LNA_EN, PINCC26XX_MUX_RFC_GPO0);
    PINCC26XX_setMux(rfx1010PinHandle, Board_PA_EN, PINCC26XX_MUX_RFC_GPO1);

    pinMode(pinHgm, OUTPUT);
    digitalWrite(pinHgm, HIGH);

    // set up LEDs
    pinMode(pinLed0, OUTPUT);
    pinMode(pinLed1, OUTPUT);
    digitalWrite(pinLed0, LOW);
    digitalWrite(pinLed1, LOW);

    OSCClockLossEventDisable();    
    OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_RCOSC_LF);
    
    // set up SPI UART
    spiuart.begin(57600);  

    spiuart.println("IRRI AutoMonPH Gateway - Functional Test");
    
    // start the RTC
    rtc.begin();
        
    // setup serial port
    Serial.begin(9600);
    
    /* Disable UART FIFOs */
    HWREG(UART0_BASE + UART_O_LCRH) &= ~UART_LCRH_FEN;
    
    // init SIM800
    gsm.begin(&Serial, &spiuart);
    
    // set up GSM
    gsm.begin(&Serial, &spiuart);
    
    // get ieee address of device
    EasyLink_getIeeeAddr(thisAddr);

    // set gateway address
    EasyLink_enableRxAddrFilter(gatewayAddrFilter, 1, 1);

    // Set the destination address
    txPacket.dstAddr[0] = DEST_NODE_ADDR;    

    // setup watchdog timer
    wdtInit();

    // reset LED sequence
    digitalWrite(pinLed0, HIGH);
    delay(200);
    digitalWrite(pinLed0, LOW);
    digitalWrite(pinLed1, HIGH);
    delay(200);
    digitalWrite(pinLed1, LOW);    
}

/**************************************************************************/
// LOOP
/**************************************************************************/
void loop() 
{
//    ts_t ts;
    uint8_t rxdata[PACKET_MAXSIZE];
    uint8_t *prxdata = rxdata;
    char pktHeader[PACKET_MAXSIZE];
    char pktText[NUMPACKETS][PACKET_MAXSIZE]; 
    char pktText2[NUMPACKETS][PACKET_MAXSIZE]; //8April2024
    char pktText3[NUMPACKETS][PACKET_MAXSIZE]; //11April2024
    uint32_t seq;
    
    // kick the dog
    Watchdog_clear(watchdogHandle);
    
    rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT_MS);
    // Turn the receiver on immediately
    rxPacket.absTime = EasyLink_ms_To_RadioTime(0);
    EasyLink_Status status = myLink.receive(&rxPacket);
    
    if (status == EasyLink_Status_Success) 
    {   
        
        digitalWrite(pinLed0, HIGH);
        memcpy(prxdata, rxPacket.payload, rxPacket.len);
        spiuart.print("Packet received with length ");
        spiuart.print(rxPacket.len);
        spiuart.print(", RSSI: ");
        spiuart.println(rxPacket.rssi); 

        packet[pktIndex].rssi = rxPacket.rssi;

        memcpy(&seq, prxdata, sizeof(seq));
        prxdata += sizeof(seq);

        // debug printout to show sequence of transmission
        sprintf(buf, "Sequence: %d - ", seq);
        spiuart.print(buf);
        for (int i=0; i<rxPacket.len; i++)
        {
            sprintf(buf, "%02X ", prxdata[i]);
            spiuart.print(buf);
        }
        spiuart.println(); 
        
        // parse the data
        memcpy(packet[pktIndex].addr, prxdata, sizeof(packet[pktIndex].addr));
        prxdata += sizeof(packet[pktIndex].addr);

        for (int i=0; i<3; i++)
        {
            memcpy(&packet[pktIndex].dist[i], prxdata, sizeof(packet[pktIndex].dist[i]));
            prxdata += sizeof(packet[pktIndex].dist[i]);
        }
    
        memcpy(&packet[pktIndex].batt, prxdata, sizeof(packet[pktIndex].batt));
        prxdata += sizeof(packet[pktIndex].batt);

        memcpy(&packet[pktIndex].retries, prxdata, sizeof(packet[pktIndex].retries));

        // sending ACK response
        // sending the nodeAddr back to the node. If any other devices hear this, they 
        // will check to see if it matches their address. If it does, then they will consider it 
        // an acknowledgement.
        memcpy(&txPacket.payload, packet[pktIndex].addr, sizeof(packet[pktIndex].addr));
        txPacket.len = 8;
        txPacket.absTime = EasyLink_ms_To_RadioTime(0);
        status = myLink.transmit(&txPacket);           

#if (DUPECHECK_ENABLE == 1)
        // check for duplicate
        if (dupeCheck(packet[pktIndex].addr, seq) != 0) 
        {
            sprintf(buf, "Duplicate packet detected. Addr: %0X, Seq: %d.\n", packet[pktIndex].addr, seq);
            spiuart.print(buf);
            return;  
        }
                  
#endif                     
            
        // get timestamp
        rtc.getTime(&packet[pktIndex].timestamp);
        
        // get temperature
        packet[pktIndex].temperature = rtc.getTemperature();

        // data consolidation
        // if we've received the consolidated packet number, send out the SMS
        if (pktIndex == NUMPACKETS-1)//changed from  1 to 0 not affecting GPRS (19March2024)
        {
            pktIndex = 1;//from 0 to 1 for alternate regular interval (20March2024)

            memset(pktHeader, 0, sizeof(pktHeader));   
            sprintf(pktHeader, "%s,%02X%02X,%02s,", FUNCTIONALMODE, thisAddr[6], thisAddr[7], gsm.signalQuality());

            for (int i=1; i<NUMPACKETS; i++)//i to 1 from 0 to send data info once only (12March2024); better 1 for GPRS (19March2024)
            {
              // memset(pktText[i], 0, sizeof(pktText[i]));   
                
//                sprintf(pktText[i], "%02X%02X,%d,%d,%d,%4.2f,%d,%d,%02d,%02d:%02d:%02d:%02d:%02d:%02d,%d,",\
//                                      packet[i].addr[6], packet[i].addr[7],  \
//                                      packet[i].dist[0],          \
//                                      packet[i].dist[1],          \
//                                      packet[i].dist[2],           \
//                                      packet[i].batt,             \
//                                      packet[i].rssi,    \
//                                      0,                \
//                                      round(packet[i].temperature),      \
//                                      packet[i].timestamp.year_s,          \
//                                      packet[i].timestamp.mon,           \
//                                      packet[i].timestamp.mday,        \
//                                      packet[i].timestamp.hour,           \
//                                      packet[i].timestamp.min,           \
//                                      packet[i].timestamp.sec,          \
//                                      packet[i].retries
//                                      );         

 //sprintf(pktText[i], "%02X%02X,%d,%4.2f ", packet[i].addr[6], packet[i].addr[7],packet[i].dist[2],packet[i].batt);//just node ID, 3rd WL reading, battery level (12March2024)  
 //sprintf(pktText[i], "%d,%4.2f,%02X%02X  ",packet[i].dist[2], packet[i].batt, packet[i].addr[6],packet[i].addr[7]);//just 3rd WL reading (read by ThingSpeak), battery level, node ID, (12March2024)  
 //sprintf(pktText2[i], "%d",packet[i].dist[2]);//8April2024 water level data
 
 sprintf(pktText[i], "%d",packet[i].dist[2]);//just 3rd WL reading (8April2024)
 sprintf(pktText2[i], "%4.2f",packet[i].batt);//11April2024 battery level data
 //sprintf(pktText2[i], "%d,%4.2f,%02X%02X  ",packet[i].dist[2], packet[i].batt, packet[i].addr[6],packet[i].addr[7]);
 sprintf(pktText3[i], "%02X%02X",packet[i].addr[6],packet[i].addr[7]);//11April2024 Node Info
            }
                
            // dump it to console to check over the format
            spiuart.println("---------------------------- SENT VIA SMS ------------------------");
/*
            spiuart.println("Un-concatenated format:");
            spiuart.print("Header: ");
            spiuart.println(pktHeader);
            for (int i=0; i<3; i++)
            {
                spiuart.print("Packet ");
                spiuart.print(i);
                spiuart.print(": ");
                spiuart.println(pktText[i]);
            }
*/            
           // sprintf(buf, "%s%s%s\n", pktHeader, pktText[0], pktText[1]);
           // sprintf(buf, "%s%s\n", pktText[0], pktText[1]);//removed header info (12March2024)
            //sprintf(buf, "%s%s\n", pktText2[0], pktText2[1]);//8April2024
           // sprintf(buf2, "%s%s\n", pktText2[0], pktText2[1]);//8April2024
            sprintf(buf1, "%s\n",pktText[1]);//11April2024 changed format specifier from %s 12April2024
           // sprintf(buf1, "%d\n",pktText[1]);//12April2024 
            sprintf(buf2, "%s\n", pktText2[1]);//11April2024 removed pktText2[0] and data is still read; changed format specifier from %s 12April2024
            sprintf(buf3, "%s\n", pktText3[1]);//11April2024 changed format specifier from %s 12April2024

//              float x = atoi(buf1); //12April2024
//              float y = atoi(buf2);//12April2024
////
//            
           // spiuart.print(pktText[1]);//12April2024;
            //int x, y, z;
            
            spiuart.print(buf3); //11April2024
            spiuart.print(buf2); //11April2024
            spiuart.print(buf1);//8April2024

            //x = (int)buf2;

           


           
                        
#if (TESTMODE == 0)
            // send via SMS
            //if (smsSend1(buf1))
            //if (smsSend(buf))
           // if (smsSend(buf1)&& smsSend(buf2)&& smsSend(buf))//8April2024 &  12April2024 multiple conditions
            if (smsSend(buf3)|| smsSend(buf2)|| smsSend(buf1))//12April2024 multiple conditions
           // if (smsSend(buf2))//11April2024 commented out either of the three to resolve duplicate sending- two data, battery & water did not disappear
            {
                spiuart.println("SUCCESS! SMS sent.");
            }
            else
            {
                spiuart.println("ERROR! SMS not sent successfully.");
                
                // how do we want to handle a failed SMS?
            }      
#endif           
        }
        else
        {
            pktIndex++;
        }
    digitalWrite(pinLed0, LOW);  
    } 
    else 
    {
        digitalWrite(pinLed0, HIGH);
        spiuart.print("Error receiving packet with status code: ");
        spiuart.print(status);
        spiuart.print(" (");
        spiuart.print(myLink.getStatusString(status));
        spiuart.println(")");
        digitalWrite(pinLed0, LOW);
    }
}

/*****************************************************************************/
// smsSend
// Send SMS message
/*****************************************************************************/

//boolean smsSend1(char *msg1)

int x; //11April2024 dummy data
//double y; //11April2024 dummy data
int z, w;
//int z= 30; //11April2024 dummy data
//int x = atoi(buf2);
//
//int x = buf1;
 
//char x[10];  //12April2024 dummy data

 



boolean smsSend(char *msg)
{
    char command[100];

    Serial.flush();
    Serial.print("AT+CMGF=1\r\n");
    spiuart.print("AT+CMGF=1\r\n");
    delay(500);
    sprintf(command, "AT+CMGS=\"%s\"\r\n", SMS_PHONE);
    Serial.print(command);
    spiuart.print(command);
  

//    Serial.print("Water Level, Battery Level, Node ID:"); //12 March 2024 to add SMS description to sensor data
//    spiuart.println("Water Level, Battery Level, Node ID:");//12 March 2024 to add SMS description to sensor data

 
   // int *a = &x;
  //  Serial.print("The pointer value is %p\n", a); //(12April2024)
   // spiuart.print("The pointer value is %p\n", a); //(12April2024)
    
    Serial.print("Node ID: "); //( Single Data - Node ID, 11April2024)
    spiuart.println ("Node ID: ");//( Single Data - Node ID, 11April2024)
    delay(1000);   
    Serial.print(buf3); //(11April2024)
    spiuart.print(buf3); //(11April2024)

    Serial.print("Water Level: "); //( Single Data - WL, 8April2024)
    spiuart.println ("Water Level: ");//( Single Data - WL, 8April2024)
    delay(1000);   
    Serial.print(buf1); //(11April2024)
    spiuart.print(buf1); //(11April2024)
    x = atof (buf1);//converting char buf1 to float for ThingSpeak access 12April2024
    Serial.print("Battery Level: "); //( Single Data - WL, 11April2024)
    spiuart.println ("Battery Level: ");//( Single Data - WL, 11April2024)
    Serial.print(buf2);
    spiuart.print(buf2);

    int y = atof (buf2); //converting char buf1 to float for ThingSpeak access 12April2024

   // y = float (y + 0.00);
    
//    float q = y /1.000;
//    y =  q /1.000;


 
     
    delay(2000); 
    Serial.write(0x1A);
    spiuart.write(0x1A);

    //x=buf1; //12April2024

 //int x = (int)buf2;


    // Serial.flush();
    
//SOL For ThingSpeak Connection Work Fine(11March2024)
Serial.flush();

//establishing GPRS internet connection (started 5 March 2024)
    Serial.print("AT+CFUN=1\r\n"); // Enable full functionality
    spiuart.print("AT+CFUN=1\r\n"); 
    delay(1000);
    Serial.print("AT+CIPMUX=0\r\n"); // Set single connection
    spiuart.print("AT+CIPMUX=0\r\n");
    delay(1000);
    Serial.print("AT+CGATT=1\r\n"); // Attach to GPRS service
    spiuart.print("AT+CGATT=1\r\n");
    delay(1000);
    Serial.print("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n"); // Set connection type to GPRS
    spiuart.print("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
    delay(1000);
    Serial.print("AT+SAPBR=3,1,\"APN\",\"internet.globe.com.ph\"\r\n"); // Set APN
    spiuart.print("AT+SAPBR=3,1,\"APN\",\"internet.globe.com.ph\"\r\n");
    delay(1000);
    Serial.print("AT+SAPBR=1,1 \r\n"); // Enable GPRS
    spiuart.print("AT+SAPBR=1,1 \r\n"); // Enable GPRS
    delay(1000);
    Serial.print("AT+SAPBR=2,1\r\n"); // Get IP address
    spiuart.print("AT+SAPBR=2,1\r\n");
    delay(1000);

    
   //send sensor data to ThingSpeak (started 6 March 2024)
   Serial.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n");
   spiuart.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n");
   delay(6000);

   Serial.print("AT+CIPSEND\r\n");
   spiuart.print("AT+CIPSEND\r\n");
   delay(4000);

//   String str="GET https://api.thingspeak.com/update?api_key=HPF57WQG0Y6IKB6K&field1=" + String(msg);   
//   Serial.println(str);
//   spiuart.println(str);

  //String str1="GET https://api.thingspeak.com/update?api_key=43GYIJ1GJN55CBKP&field1=" + String(buf2)+"&field2="+ String(buf1); 
  //String str="GET https://api.thingspeak.com/update?api_key=1TULQDLNEVL8ZI2E&field1=" + String(surfaceWL)+"&field2="+ String(avgWl) +"&field3="+ String(battVolt)+ "&field4="+ String(stat); 
  //String str1="GET https://api.thingspeak.com/update?api_key=43GYIJ1GJN55CBKP&field1=" + String(buf1)+"&field2="+ String(buf2); //11April2024
   //String str="GET https://api.thingspeak.com/update?api_key=43GYIJ1GJN55CBKP&field1=" + String(buf1)+"&field2="+ String(buf2) +"&field3="+ String(buf); 
 
   //String str="GET https://api.thingspeak.com/update?api_key=43GYIJ1GJN55CBKP&field1=" + String(z)+"&field2="+ String(x)+"&field3="+ String(y)+"&field4="+ String(buf);//added char converted x & y in 12 April 2024 did work
   String str="GET https://api.thingspeak.com/update?api_key=43GYIJ1GJN55CBKP&field1=" + String(x)+"&field2="+ String(y)+"&field3="+ String(buf3); //15April 2024 removed the placement holder z
   Serial.println(str);
   spiuart.println(str);
   delay(4000);
  
   Serial.write(0x1A);
   spiuart.write(0x1A);
    
//   Serial.write(0x1A);
//   spiuart.write(0x1A);

//EOL For ThingSpeak Connection (11March2024)



  

//if(BUILD_PHILRICE == 1)
//{
//    delay(5000);   
//    Serial.flush();
//    Serial.print("AT+CMGF=1\r\n");
//    spiuart.print("AT+CMGF=1\r\n");
//    delay(500);
//    sprintf(command, "AT+CMGS=\"%s\"\r\n", SMS_PHONE2);
//    Serial.print(command);
//    spiuart.print(command);
//    Serial.print(msg);
//    spiuart.print(msg);
//    delay(2000); 
//    Serial.write(0x1A);
//    spiuart.write(0x1A);

//}  
    return 1;
/*
  if (len > 140) 
  {
        spiuart.println("ERROR: Message too big");
  }
  
  return gsm.sendSms((char *)number,  (char *)msg);
*/
}

/**************************************************************************/
// dupeCheck
/**************************************************************************/
uint8_t dupeCheck(uint8_t *addr, uint32_t seq)
{
    for (int i=0; i<DUPETABLE_MAX_ENTRIES; i++)
    {
        if (memcmp(dupeTable[i].addr, addr, 8) == 0)
        {
            if (dupeTable[i].seq == seq)
            {
                return 1;
            }
            
        }
    }

    // address/seq combo wasn't found. add to dupeTable
    // first shift all dupe entries forward
    for (int i=0; i<DUPETABLE_MAX_ENTRIES-1; i++)
    {
        memcpy(&dupeTable[i+1], &dupeTable[i], sizeof(dupe_t));
    }

    // add new entry to 0th element
    memcpy(dupeTable[0].addr, addr, 8);
    dupeTable[0].seq = seq;
    
    return 0;
}

/*****************************************************************************/
// Setup Watchdog timer
/*****************************************************************************/
void wdtInit()
{
    Watchdog_init();
    Watchdog_Params_init(&params);
    params.callbackFxn = (Watchdog_Callback) watchdogCallback;
    watchdogHandle = Watchdog_open(Board_WATCHDOG, &params);
    if (watchdogHandle == NULL)
    {
          Serial.println("Error opening watchdog timer handle.");
          // Error opening Watchdog
          while (1) {}
    }
    reloadValue = Watchdog_convertMsToTicks(watchdogHandle, WD_TIMEOUT_MS);
    Watchdog_setReload(watchdogHandle, reloadValue);
}

/**************************************************************************/
/**************************************************************************/
void watchdogCallback(uintptr_t watchdogHandle)
{
    /*
     * If the Watchdog Non-Maskable Interrupt (NMI) is called,
     * loop until the device resets. Some devices will invoke
     * this callback upon watchdog expiration while others will
     * reset. See the device specific watchdog driver documentation
     * for your device.
     */
    while (1) {}
}
