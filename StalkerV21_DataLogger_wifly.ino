//STALKER + WIFLY Low power
//Send temp and batt to Ubidots
//Author: Rodrigo Carbajales MarconiLab

#include <MemoryFree.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "WiFlyHQ.h"
#include<stdlib.h>  //it is a must 4 float 2 String (dtostrf)
//dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/power.h>
#include <Wire.h>
#include <DS3231.h>
//#include <Fat16.h>
//#include <Fat16util.h>

//The following code is taken from sleep.h as Arduino Software v22 (avrgcc), in w32 does not have the latest sleep.h file
#define sleep_bod_disable() \
{ \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
}


DS3231 RTC; //Create RTC object for DS3231 RTC come Temp Sensor 
char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
//year, month, date, hour, min, sec and week-day(starts from 0 and goes to 6)
//writing any non-existent time-data may interfere with normal operation of the RTC.
//Take care of week-day also.
DateTime dt(2014, 06, 05, 11, 5, 00, 4);

char CH_status_print[][4]=  { "Off","On ","Ok ","Err" };

static uint8_t prevSecond=0; 
static uint8_t prevMinute=0; 
static DateTime interruptTime;
int resetTimer = 0; 

//SdCard card;
//Fat16 file;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

/*void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}*/

/* Change these to match your WiFi network */
const char mySSID[] = "MarconiLab";
const char myPassword[] = "marconi-lab";
#define AUTH      WIFLY_AUTH_WPA2_PSK     // or WIFLY_AUTH_WPA1, WIFLY_AUTH_WEP, WIFLY_AUTH_OPEN

//Dots configuration
#define TOKEN          "YRWsXoUOptheqaDFN2T58o65hyHe4m"  //Replace with your TOKEN
#define VARIABLEID1    "539eec807625422e808b2a92"
#define VARIABLEID2    "539eebc27625422e1a16dfd3"       //Replace with your variable ID
char server[] = "things.ubidots.com";
//char server[] = "192.168.88.119";  //TCPMON

// Arduino       WiFly TX:6 RX:7
SoftwareSerial wifiSerial(6,7);
WiFly wifly;

char buf[80];

void setup () 
{
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04; 
     DDRD &=~ 0x04;
     pinMode(4,INPUT);//extern power
   
     Wire.begin();
     Serial.begin(57600);
     RTC.begin();
     RTC.adjust(dt); //Adjust date-time as defined 'dt' above 
     //delay(1000)
     attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     //Enable Interrupt 
     RTC.enableInterrupts(EveryMinute); //interrupt at  EverySecond, EveryMinute, EveryHour
     analogReference(INTERNAL); //Read battery status
     
/*Wifly memory*/
    Serial.println(F("Starting"));
    Serial.print(F("Free Wifly memory: "));
    Serial.println(wifly.getFreeMemory(),DEC);

/*Start Wifly*/
    wifiSerial.begin(9600);
    if (!wifly.begin(&wifiSerial, &Serial)) {
        Serial.println(F("Failed to start wifly"));
	wifly.terminal();
    }

/* Join wifi network if not already associated */
   if (!wifly.isAssociated()) {
	/* Setup the WiFly to connect to a wifi network */
	Serial.println(F("Joining network"));
	wifly.setSSID(mySSID);
	wifly.setPassphrase(myPassword);
	wifly.enableDHCP();
	wifly.save();

	if (wifly.join()) {
	    Serial.println(F("Joined wifi network"));
	} else {
	    Serial.println(F("Failed to join wifi network"));
	    wifly.terminal();
	}
    } else {
        Serial.println(F("Already joined network"));
    }

    wifly.setBroadcastInterval(0);	// Turn off UPD broadcast

    //wifly.terminal();

/*print MAC & IP*/
   Serial.print(F("MAC: "));
    Serial.println(wifly.getMAC(buf, sizeof(buf)));
    Serial.print(F("IP: "));
    Serial.println(wifly.getIP(buf, sizeof(buf)));

/*Set device ID*/
    wifly.setDeviceID("Wifly-WebServer");

    if (wifly.isConnected()) {
        Serial.println(F("Old connection active. Closing"));
	wifly.close();
    }

/*TCP connection*/
    wifly.setProtocol(WIFLY_PROTOCOL_TCP);
    if (wifly.getPort() != 80) {
        wifly.setPort(80);
	/* local port does not take effect until the WiFly has rebooted (2.32) */
        wifly.save();
	Serial.println(F("Set port to 80, rebooting to make it work"));
	wifly.reboot();
	delay(3000);
    }
    Serial.println(F("Ready"));
}

void loop () 
{
    Serial.print(F("Free RAM Memory="));
    Serial.println(freeMemory());
    
    //Battery Charge Status and Voltage reader
    int BatteryValue = analogRead(A7);
    float voltage=BatteryValue * (1.1 / 1024)* (10+2)/2;  //Voltage devider
    unsigned char CHstatus = read_charge_status();//read the charge status
    
    // Convert battery voltage to string for UbiDots
    char buffer3[14]; //make buffer large enough for 7 digits
    String voltageS = dtostrf(voltage,7,2,buffer3);
    //'7' digits including '-' negative, decimal and white space. '2' decimal places
    voltageS.trim(); //trim whitespace, important so ThingSpeak will treat it as a number
  
    ////////////////////// START : Application or data logging code//////////////////////////////////
    RTC.convertTemperature();          //convert current temperature into registers
    float temp = RTC.getTemperature(); //Read temperature sensor value
    
    // Convert temp to string for UbiDots
    char buffer4[14]; //make buffer large enough for 7 digits
    String tempS = dtostrf(temp,7,2,buffer4);
    //'7' digits including '-' negative, decimal and white space. '2' decimal places
    tempS.trim(); //trim whitespace, important so ThingSpeak will treat it as a number
    
    DateTime now = RTC.now(); //get the current date-time    
//    if((now.second()) !=  prevSecond )
    if((now.minute()) !=  prevMinute )
    {
    //print only when there is a change
    Serial.print(now.year(), DEC);
    Serial.print(F("/"));
    Serial.print(now.month(), DEC);
    Serial.print(F("/"));
    Serial.print(now.date(), DEC);
    Serial.print(F(" "));
    Serial.print(now.hour(), DEC);
    Serial.print(F(":"));
    Serial.print(now.minute(), DEC);
    Serial.print(F(":"));
    Serial.print(now.second(), DEC);
    Serial.print(F(" "));
    Serial.print(weekDay[now.dayOfWeek()]);
    Serial.print(F(" "));
    Serial.print(temp);
    Serial.write(186);
    Serial.print(F("C"));
    Serial.print(F(", "));
    Serial.print("Battery Voltage -> ");
    Serial.print(voltage);
    Serial.print(F("V, "));
    Serial.print(F("Charge status --> "));
    Serial.println(CH_status_print[CHstatus]);
    }
     prevSecond = now.second();
     prevMinute = now.minute();
     
 /*   //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||
    // initialize the SD card
    if (!card.init()) error("card.init");
  
    // initialize a FAT16 volume
    if (!Fat16::init(&card)) error("Fat16::init");
  
    char name[] = "TEMP.CSV";
    // clear write error
    file.writeError = false;
  
    // O_CREAT - create the file if it does not exist
    // O_APPEND - seek to the end of the file prior to each write
    // O_WRITE - open for write
    if (!file.open(name, O_CREAT | O_APPEND | O_WRITE))
        error("error opening file");

    file.print(now.year(), DEC);
    file.print('/');
    file.print(now.month(), DEC);
    file.print('/');
    file.print(now.date(), DEC);
    file.print(',');
    file.print(now.hour(), DEC);
    file.print(':');
    file.print(now.minute(), DEC);
    file.print(':');
    file.print(now.second(), DEC);
    file.print(',');
    file.print(temp);
    file.print(',');
    file.print(voltage);
    file.print(',');
    file.println(CH_status_print[CHstatus]);

    if (!file.close()) 
        error("error closing file");
    //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||*/
    
     Serial.println("Trying to send data: " + voltageS+"V");
     writeUbidots(tempS, VARIABLEID1 );     //Send data in String format to the Ubidots function
     writeUbidots(voltageS, VARIABLEID2 );     //Send data in String format to the Ubidots function
      if(resetTimer == 10) {                    //Restarts if the connection fails 10 times in a row
        asm volatile ("  jmp 0");
      }
    wifly.sleep(60);
       
    
    RTC.clearINTStatus(); //This function call is  a must to bring /INT pin HIGH after an interrupt.
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    
    
    ////////////////////////END : Application code //////////////////////////////// 
   
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
    //Power Down routines
    cli(); 
    sleep_enable();      // Set sleep enable bit
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
        
    Serial.println("\nSleeping");
    delay(50); //This delay is required to allow print to complete
    //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); //This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)  
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  //This shuts enables ADC, TWI, SPI, Timers and USART
    delay(10); //This delay is required to allow CPU to stabilize
    Serial.println("Awake from sleep");    
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 

unsigned char read_charge_status(void)
{
  unsigned char CH_Status=0;
  unsigned int ADC6=analogRead(6);
  if(ADC6>900)
  {
    CH_Status = 0;//sleeping
  }
  else if(ADC6>550)
  {
    CH_Status = 1;//charging
  }
  else if(ADC6>350)
  {
    CH_Status = 2;//done
  }
  else
  {
    CH_Status = 3;//error
  }
  return CH_Status;
}
  
//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{

  //Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    Serial.println("Interruption");
}

//UBIDOTS
void writeUbidots(String data, String VARID) {
  String dataString = "{\"value\":"+ data + "}";                  //Prepares the data in JSON format

  if (wifly.open(server, 80)) {                               //If connection is successful, then send this HTTP Request:
    Serial.println("Connected to " + String(VARID));
    wifly.print("POST /api/v1.6/variables/");                    //Specify URL, including the VARIABLE ID
    wifly.print(VARID);
    wifly.println("/values HTTP/1.1");
    wifly.println("Host: things.ubidots.com");
    wifly.print("X-Auth-Token: ");                               //Specify Authentication Token in headers
    wifly.println(TOKEN);
    wifly.println("Content-Type: application/json");
    wifly.print("Content-Length: ");
    wifly.println(dataString.length());
    //   wifly.println("Connection: close");
    wifly.println();                                             //End of HTTP headers

    wifly.println(dataString);                                   //This is the actual value to send

    wifly.flush();
    wifly.close();
//    return 1;
  }
  else {                                                          // If the connection wasn't possible, then:
    resetTimer += 1;
    Serial.println("Connection failed");
    Serial.println("Device will restart after 10 failed attempts, so far:"+String(resetTimer)+" attempts.");
    Serial.println("Killing sockets and disconnecting...");
    wifly.flush();
    wifly.close();
//    return 0;
  }
}

