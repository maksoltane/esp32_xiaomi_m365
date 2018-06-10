//russische definitionen: https://github.com/fogbox/m365_display/blob/master/extend_speedometer/defines.h & https://electro.club/forum/elektrosamokatyi_xiaomi/displey_dlya_syaokata&page=8
//spanische definitionen: https://github.com/CamiAlfa/M365-BLE-PROTOCOL/blob/master/m365_register_map.h


/*next steps:
- fix missing crlf in serial debug
- add 36524 for raw data dump
- verify structs/data
	-> decoder finished, start with requestor

*/
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <Update.h> //needed for arduinoOTA on esp32
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
#else
  #error Platform not supported
#endif

#include <ArduinoOTA.h>

//RESOLDER RX PIN... :D

//more ansi codes...
// http://www.termsys.demon.co.uk/vtansi.htm

//FIX: WIFI OFF Prints all the time.... :(

/*TODO: 
    rename telnet stuff to clientservices or similar and add http
*/

#define swversion "ESP32 M365 OLED\r\nv20180609"
//DEBUG Settings
  //#define debug_dump_states //dump state machines state
  //#define debug_dump_rawpackets //dump raw packets to Serial/Telnet
  #define debug_dump_packetdecode //dump infos from packet decoder

//#define usemqtt
#ifdef usemqtt
  #include <PubSubClient.h>
  #define mqtt_clientID "m365"
  #define mqtt_server "192.168.0.31"
  #define mqtt_port 1883
#endif
#define OTApwd "h5fj8ovbthrfd65b4"

//OLED
  /* Modifications to ADAFRUIT SSD1306 Library 1.02for ESP32:
   *  Adafruit_SSD1306.h:35 - #elif defined(ESP8266) || defined(ESP32) || defined(ARDUINO_STM32_FEATHER)
   *  Adafruit_SSD1306.cpp:27 - #if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) && !defined(ESP32)
   */
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define sclpin GPIO_NUM_15
  #define sdapin GPIO_NUM_4 
  #define OLED_RESET 16 //original war das auch 4, -1 geht auch -> ist gpio nummer... somit nicht belegt...
  Adafruit_SSD1306 display(OLED_RESET);
  //Standard font, display.setTextSize(1); = 8x8pix (Char = 7x7, 1 blank right/bottom
  #define line1 0
  #define line2 8
  #define line3 16
  #define line4 32
  #define line5 48
  #define line6 56
//WLAN
  #define ssid1 "m365dev"
  #define password1 "h5fj8bvothrfd65b4"
  #define ssid2 "..."
  #define password2 "..."
  #define ssid3 "..."
  #define password3 "..."

  const char *apssid ="m365oled";
  const char *appassword="365";
  //#define staticip
  #ifdef staticip
    IPAddress ip(192,168,0,149);  //Node static IP
    IPAddress gateway(192,168,0,1);
    IPAddress dns(192,168,0,1);
    IPAddress subnet(255,255,255,0);
  #endif
  
  uint8_t wlanstate = 0;
  uint8_t wlanstateold = 0;
  #define wlanoff 0
  #define wlanturnapon 1
  #define wlanturnstaon 2
  #define wlansearching 3
  #define wlanconnected 4
  #define wlanap 5
  #define wlanturnoff 6 

  unsigned long wlanconnecttimestamp = 0;
  #define wlanconnecttimeout 10000 //timeout for connecting to one of  the known ssids
  #define wlanapconnecttimeout 60000 //timeout in ap mode until client needs to connect / ap mode turns off
  uint8_t currentssidindex = 0;
  #define maxssids 1
  uint8_t apnumclientsconnected=0;
  uint8_t apnumclientsconnectedlast=0;

  unsigned long userconnecttimestamp = 0;
  #define userconnecttimeout 600000 //timeout for connecting to telnet/http after wlan connection has been established with ap or client
  
  unsigned long telnetlastrefreshtimestamp = 0;
  #define telnetrefreshanyscreen 100 //refresh telnet screen every xx ms
  #define telnetrefreshrawarrayscreen 500 //refresh telnet screen every xx ms
//TELNET
  #define MAX_SRV_CLIENTS 1
  WiFiServer server(36523);
  //WiFiServer telnetserver(36523);
  //WiFiServer rawserver(36524);
  WiFiClient serverClients[MAX_SRV_CLIENTS];
  uint8_t telnetstate = 0;
  uint8_t telnetstateold = 0;
  #define telnetoff 0
  #define telnetturnon 1
  #define telnetlistening 2
  #define telnetclientconnected 3
  #define telnetturnoff 4
  #define telnetdisconnectclients 5

  //char telnetbuffer[2000];
  //ANSI
  // ansi stuff, could always use printf instead of concat
  String ansiPRE  = "\033"; // escape code
  String ansiHOME = "\033[H"; // cursor home
  String ansiESC  = "\033[2J"; // esc
  String ansiCLC  = "\033[?25l"; // invisible cursor
  String ansiEND  = "\033[0m";   // closing tag for styles
  String ansiBOLD = "\033[1m";
  String ansiRED  = "\033[41m"; // red background
  String ansiGRN  = "\033[42m"; // green background
  String ansiBLU  = "\033[44m"; // blue background
  String ansiREDF = "\033[31m"; // red foreground
  String ansiGRNF = "\033[34m"; // green foreground
  String ansiBLUF = "\033[32m"; // blue foreground
  String BELL     = "\a";

//M365
  HardwareSerial Serial1(2);  // UART1/Serial1 pins RX 16, TX17
  #define UART2RX GPIO_NUM_5
  #define UART2TX GPIO_NUM_17
  #define UART2RXunused GPIO_NUM_23 //ESP32 does not support RX or TX only modes - so we remap the rx pin to a unused gpio during sending
  //custom pins see here: http://www.iotsharing.com/2017/05/how-to-use-serial-arduino-esp32-print-debug.html?m=1
  //enable rx/tx only  modes see: https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.h
  #ifdef ESP32
    #define Serial1Full Serial1.begin(115200,SERIAL_8N1, UART2RX, UART2TX)
    //#define Serial1RX Serial1.begin(115200,SERIAL_8N1, UART2RX, -1)
    #define Serial1TX Serial1.begin(115200,SERIAL_8N1, UART2RXunused, UART2TX)
  #elif defined(ESP8266)
    #define Serial1Full Serial1.begin(115200,SERIAL_8N1, UART_FULL, UART2RX, UART2TX)
    //#define Serial1RX Serial1.begin(115200,SERIAL_8N1, UART_RX_ONLY, UART2RX, UART2TX)
    #define Serial1TX Serial1.begin(115200,SERIAL_8N1, UART_TX_ONLY, UART2RX, UART2TX)
  #endif
  
  #define maxlen 256
  uint8_t crc1=0;
  uint8_t crc2=0;
  uint16_t crccalc=0;
  uint8_t sbuf[maxlen];
  uint8_t len=0;
  uint8_t readindex=0;
  uint8_t m365receiverstate = 0;
  uint8_t m365receiverstateold = 0;
  #define m365receiveroff 0 //Serial1 will not be read...
  #define m365receiverready 1 //reading bytes until we find 0x55
  #define m365receiverpacket1 2 //preamble 0x55 received, waiting for 0xAA
  #define m365receiverpacket2 3 //len preamble received waiting for LEN
  #define m365receiverpacket3 4 //payload this state is kept until LEN bytes received
  #define m365receiverpacket4 5 //checksum receiving checksum
  #define m365receiverpacket5 6 //received a packet, test for checksum
  #define m365receiverstorepacket 7 //packet received, checksum valid, store in array, jump back to receiverready for next packet, set newpacket flag

  uint8_t m365packetstate = 0;
  uint8_t m365packetstateold = 0;
  #define m365packetidle 0
  #define m365newpacket 1

//M365 Statistics
  uint16_t packets_rec=0;
  uint16_t packets_crcok=0;
  uint16_t packets_crcfail=0;
  uint16_t packetsperaddress[256];


//OLED

//TIMERs
  //wlanconnecttimeout
  //telnetconnecttimeout
  //??

//Misc
  #define led GPIO_NUM_2
  int ledontime = 200;
  int ledofftime = 10000;
  unsigned long ledcurrenttime = 100;
  
  uint8_t i;
  char tmp1[200];
  char tmp2[200];
  uint16_t tmpi;
  char chipid[7];
  char mac[12];


//offsets in sbuf
#define i_address 0
#define i_hz 1
#define i_offset 2
#define i_payloadstart 3


#define address_ble 0x20
#define address_x1 0x21
#define address_esc 0x23
#define address_bms 0x25

//#define MastertoM365 0x20
//#define M365toMaster 0x23
//#define MastertoBATT 0x22
//#define BATTtoMaster 0x25


//request array for ble - address 0x20: offset, len
//those values do not need to be requested as ble module sends them on change
uint16_t blerequests[][2]= {
  { 1, 1}, //throttle
  { 1, 1} //brake
};

//request array for "x1" unkown device/meaning - address 0x21: offset, len
uint16_t x1requests[][2]= {
  { 0, 1}, //mode: 0-stall, 1-drive, 2-eco stall, 3-eco drive
  { 1, 1}, //batt leds: battery status 0 - min, 7(or 8...) - max
  { 2, 1}, //light 0= off, 0x64 = on
  { 3, 1} //"beepaction" ?
};



//request array for esc - address 0x23: offset, len
uint16_t escrequests[][2]= {
  { 0x25, 2}, //RemainingMilage/100 =km
  { 0x3A, 2}, //Power on Time in Seconds
  { 0x3C, 2}, //Riding Time in Seconds
  { 0xBA, 2}, //current speed / 1000 in km/h
  { 0xBC, 2}, //Average Speed/1000 in km/h
  { 0xBE, 4}, //TotalMileage/1000
  { 0xC2, 2}, //CurrentMilage/1000
  { 0xC4, 2}, //Powerontime in seconds
  { 0x3E, 2}, //mainframe temp (/10?)
  { 0xC6, 2}, //MainframeTemp/10
  { 0x7B, 1}, //recuperation:  0 - weak, 1 - medium, 2 - strong
  { 0x7C, 1}, //cruise mode: 0 - cruise mode OFF; 1 - cruise mode ON
  { 0x7D, 1} //rear light: 2 = ON, 0 = OFF
};

//request array for bms - address 0x25: offset, len
uint16_t bmsrequests[][2]= {
  { 0x31, 2}, //remaining mAh
  { 0x33, 1}, //percent remaining
  { 0x34, 1}, //battery status?
  { 0x35, 2}, //current        /100 = A
  { 0x38, 2}, //batt voltage   /100 = V
  { 0x3A, 1}, //temp 1 (-20 = °C)
  { 0x3B, 1}, //temp 2 (-20 = °C)
  { 0x40, 2}, //Voltage/1000 Cell 1
  { 0x42, 2}, //Voltage/1000 Cell 2
  { 0x44, 2}, //Voltage/1000 Cell 3
  { 0x46, 2}, //Voltage/1000 Cell 4
  { 0x48, 2}, //Voltage/1000 Cell 5
  { 0x4A, 2}, //Voltage/1000 Cell 6
  { 0x4C, 2}, //Voltage/1000 Cell 7
  { 0x4E, 2}, //Voltage/1000 Cell 8
  { 0x50, 2}, //Voltage/1000 Cell 9
  { 0x52, 2}, //Voltage/1000 Cell 10
  { 0x54, 2}, //Voltage/1000 Cell 11
  { 0x56, 2}, //Voltage/1000 Cell 12
  { 0x58, 2}, //Voltage/1000 Cell 13
  { 0x5A, 2}, //Voltage/1000 Cell 14
  { 0x5C, 2}, //Voltage/1000 Cell 15
};


typedef struct {
  uint8_t u1[1];
  uint8_t throttle;
  uint8_t brake;
  uint8_t u2[509];
} blestruct;

typedef struct {
  uint8_t mode; //offset 0x00 mode: 0-stall, 1-drive, 2-eco stall, 3-eco drive
  uint8_t battleds;  //offset 0x01 battery status 0 - min, 7(or 8...) - max
  uint8_t light;  //offset 0x02 0= off, 0x64 = on
  uint8_t beepaction;  //offset 0x03 "beepaction" ?
  uint8_t u1[508];
} x1struct;

typedef struct {
  uint16_t u1[10]; //offset 0-0x1F
  unsigned char serial[14]; //offset 0x20-0x2D
  unsigned char pin[6]; //offset 0x2E-0x33
  uint16_t fwversion; //offset 0x34-035,  e.g. 0x133 = 1.3.3
  uint16_t u2[10]; //offset 0x36-0x49
  uint16_t remainingdistance; //offset 0x4a-0x4B e.g. 123 = 1.23km
  uint16_t u3[14]; //offset 0x4C-0x75
  uint16_t triptime; //offset 0x76-0x77 trip time in seconds
  uint16_t u4[2]; //offset 0x4C-0x75
  uint16_t frametemp1; //offset 0x7C-0x7D /10 = temp in °C
  uint16_t u5[36]; //offset 0x7e-0xe9
  uint16_t ecomode; //offset 0xEA-0xEB; on=1, off=0
  uint16_t u6[10]; //offset 0xec-0xf5
  uint16_t kers; //offset 0xf6-0xf7; 0 = weak, 1= medium, 2=strong
  uint16_t cruisemode; //offset 0xf8-0xf9, off 0, on 1
  uint16_t taillight; //offset 0xfa-0xfb, off 0, on 2
  uint16_t u7[50]; //offset  0xfc-0x15f
  uint16_t error; //offset 0x160-0x161
  uint16_t u8[3]; //offset 0x162-0x167
  uint16_t battpercent; //offset 0x168-0x169
  uint16_t speed; //0x16A-0x16B meter per second?
  uint16_t averagespeed; //0x16C-0x16D meter per second?
  uint32_t totaldistance; //0x16e-0x171
  uint16_t tripdistance; //0x172-0x173
  uint16_t u9[1]; //offset 0x174-0x175 
  uint16_t frametemp2; //0x176-0x177 /10 = temp in °C
  uint16_t u10[68]; //offset 0x178-0x200 
} escstruct;

typedef struct {
  uint16_t u1[10]; //offset 0-0x1F
  unsigned char serial[14]; //offset 0x20-0x2D
  uint16_t fwversion; //offset 0x2E-0x2f e.g. 0x133 = 1.3.3
  uint16_t totalcapacity; //offset 0x30-0x31 mAh
  uint16_t u2[7]; //offset 0x32-0x3f
  uint16_t proddate; //offset 0x40-0x41
      //fecha a la batt 7 MSB->año, siguientes 4bits->mes, 5 LSB ->dia ex:
    //b 0001_010=10, año 2010
    //        b 1_000= 8 agosto
    //            b  0_1111=15 
    //  0001_0101_0000_1111=0x150F 
  uint16_t u3[0x10]; //offset 0x42-0x61
  uint16_t remainingcapacity; //offset 0x62-0x63
  uint16_t remainingpercent; //offset 0x64-0x65
  int16_t current; //offset 0x66-67 - negative = charging; /100 = Ampere
  uint16_t voltage; //offset 0x68-69 /10 = Volt
  uint16_t temperature; //offset 0x6A-0x6B -20 = °C
  uint16_t u4[4]; //offset 0x6C-0x75
  uint16_t health; //offset 0x76-0x77; 0-100, 60 schwellwert "kaputt"
  uint16_t u5[4]; //offset 0x78-0x7F
  uint16_t Cell1Voltage; //offset 0x80-0x81
  uint16_t Cell2Voltage; //offset 0x82-0x83
  uint16_t Cell3Voltage; //offset 0x84-0x85
  uint16_t Cell4Voltage; //offset 0x86-0x87
  uint16_t Cell5Voltage; //offset 0x88-0x89
  uint16_t Cell6Voltage; //offset 0x8A-0x8B
  uint16_t Cell7Voltage; //offset 0x8C-0x8D
  uint16_t Cell8Voltage; //offset 0x8E-0x8F
  uint16_t Cell9Voltage; //offset 0x90-0x91
  uint16_t Cell10Voltage; //offset 0x92-0x93
  uint16_t u6[182]; //offset 0x94-0x
} bmsstruct;

uint8_t bledata[512];
uint8_t x1data[512];
uint8_t escdata[512];
uint8_t bmsdata[512];

blestruct* bleparsed = (blestruct*)bledata;
x1struct* x1parsed = (x1struct*)x1data;
escstruct* escparsed = (escstruct*)escdata;
bmsstruct* bmsparsed = (bmsstruct*)bmsdata;

bool newdata = false;

void reset_statistics() {
  packets_rec=0;
  packets_crcok=0;
  packets_crcfail=0;
  for(uint16_t j = 0; j<=255; j++) {
    packetsperaddress[j]=0;  
  }
  for(uint16_t j = 0; j<=511; j++) {
    escdata[j]=0;  
    bmsdata[j]=0;  
    bledata[j]=0;  
    x1data[j]=0;  
  }
}
void start_m365() {
  Serial1Full;
  m365receiverstate = m365receiverready;
  m365packetstate=m365packetidle;
  reset_statistics();
}  //startm365


void m365_handlepacket() {
 if (m365packetstate==m365newpacket) {
    packetsperaddress[sbuf[i_address]]++;  
    #ifdef debug_dump_packetdecode
      sprintf(tmp1,"PACKET Len %02X Addr %02X HZ %02X Offset %02X CRC %04X Payload: ",len,sbuf[i_address],sbuf[i_hz],sbuf[i_offset], crccalc);
      Serial.print(tmp1);
      for(i = 0; i < len-3; i++){
        Serial.printf("%02X ",sbuf[i_payloadstart+i]);
      }
      //Serial.println("");
      /*for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
          serverClients[i].write(tmp1,37);
          yield();
       }
      }*/
    #endif
    /*
    switch (sbuf[i_address]) {
      case address_bms:
          memcpy((void*) bmsdata[sbuf[i_offset]<<1], (void*)sbuf[i_payloadstart], len-3);
        break;
      case address_esc:
          memcpy((void*) escdata[sbuf[i_offset]<<1], (void*)sbuf[i_payloadstart], len-3);
        break;
      case address_ble:
          memcpy((void*) bledata[sbuf[i_offset]<<1], (void*)sbuf[i_payloadstart], len-3);
        break;
      case address_x1:
          memcpy((void*) x1data[sbuf[i_offset]<<1], (void*)sbuf[i_payloadstart], len-3);
        break;
    }
    */

    switch (sbuf[i_address]) {
      case address_bms:
          //Serial.printf("--> BMS start %d (sbuf offset %d, len %d)\r\n", sbuf[i_offset]<<1,sbuf[i_offset], len-3);
          memcpy((void*)& bmsdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_esc:
          //Serial.printf("--> ESC start %d (sbuf offset %d, len %d)\r\n", sbuf[i_offset]<<1,sbuf[i_offset], len-3);
          memcpy((void*)& escdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_ble:
          //Serial.printf("--> BLE start %d (sbuf offset %d, len %d)\r\n", sbuf[i_offset]<<1,sbuf[i_offset], len-3);
          memcpy((void*)& bledata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_x1:
          //Serial.printf("--> X1 start %d (sbuf offset %d, len %d)\r\n", sbuf[i_offset]<<1,sbuf[i_offset], len-3);
          memcpy((void*)& x1data[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      default:
          Serial.println("");
       break;
    }
    newdata=true;
    m365packetstate=m365packetidle;
 } //if (m365packetstate==1)
 
} //m365_handlepacket
  


void m365_receiver() { //recieves data until packet is complete
  uint8_t newbyte;
  if (Serial1.available()) {
    newbyte = Serial1.read();
  
    switch(m365receiverstate) {
      case m365receiverready: //we are waiting for 1st byte of packet header 0x55
          if (newbyte==0x55) {
            m365receiverstate = m365receiverpacket1;
          }
        break;
      case m365receiverpacket1: //we are waiting for 2nd byte of packet header 0xAA
          if (newbyte==0xAA) {
            m365receiverstate = m365receiverpacket2;
            //len=0;
            //crc1=0;
            //crc2=0;
            //crccalc=0;
          }
        break;
      case m365receiverpacket2: //we are waiting for packet length
          len = newbyte+1; //next byte will be 1st in sbuf, this is the packet-type1, len counted from 2nd byte in sbuf
          crccalc=newbyte;
          readindex=0;
          m365receiverstate = m365receiverpacket3;
        break;
      case m365receiverpacket3: //we are receiving the payload
          sbuf[readindex]=newbyte;
          readindex++;
          crccalc=crccalc+newbyte;
          if (readindex==len) {
            m365receiverstate = m365receiverpacket4;
          }
        break;
      case m365receiverpacket4: //we are waiting for 1st CRC byte
          crc1 = newbyte;
          m365receiverstate = m365receiverpacket5;
        break;
      case m365receiverpacket5: //we are waiting for 2nd CRC byte
          crc2 = newbyte;
          crccalc = crccalc ^ 0xffff;
          #ifdef debug_dump_rawpackets
            sprintf(tmp1,"Packet received: 55 AA %02X ", len-1);
            sprintf(tmp2,"CRC %02X %02X %04X\r\n", crc1,crc2,crccalc);
            Serial.print(tmp1);
            for(i = 0; i < len; i++){
             Serial.printf("%02X ",sbuf[i]);
            }
            Serial.print(tmp2);
            /*
            for(i = 0; i < MAX_SRV_CLIENTS; i++){
              if (serverClients[i] && serverClients[i].connected()){
                serverClients[i].write(tmp1,26);
                for(i = 0; i < (len); i++){
                  serverClients[i].printf("%02X ",sbuf[i]);
                  //serverClients[i].write(tmp1,3);
                }
                serverClients[i].write(tmp2, 12);
                yield();
             }
            }
            */
          #endif
          packets_rec++;
          if (crccalc==((uint16_t)(crc2<<8)+(uint16_t)crc1)) {
            m365packetstate = m365newpacket;
            packets_crcok++;
          } else {
            packets_crcfail++;
          }

          m365receiverstate = m365receiverready; //reset and wait for next packet
        break;
    } //switch
  }//serial available
} //m365_receiver
 

#define ts_telemetrie 0
#define ts_statistics 1
#define ts_esc_raw 2
#define ts_ble_raw 3
#define ts_bms_raw 4
#define ts_x1_raw 5
#define ts_esc_array 6
#define ts_ble_array 7
#define ts_bms_array 8
#define ts_x1_array 9
uint8_t telnetscreen = ts_statistics;
uint8_t telnetscreenold = telnetscreen;

void telnet_refreshscreen() {
  uint8_t k=0;
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
        if (telnetscreenold!=telnetscreen) { serverClients[i].print(ansiESC); telnetscreenold=telnetscreen; }
        serverClients[i].print(ansiHOME);
        
        serverClients[i].print(ansiCLC); 
        switch (telnetscreen) {
          case ts_telemetrie:
              serverClients[i].print("M365 Telemetrie Screen"); 
            break;
          case ts_esc_raw:
              serverClients[i].print("M365 ESC RAW Screen"); 
            break;
          case ts_ble_raw:
              serverClients[i].print("M365 BLE RAW Screen"); 
            break;
          case ts_bms_raw:
              serverClients[i].print("M365 BMS RAW Screen"); 
            break;
          case ts_x1_raw:
              serverClients[i].print("M365 X1 RAW Screen"); 
            break;
          case ts_esc_array:
              serverClients[i].print("M365 ESC Array Screen"); 
            break;
          case ts_ble_array:
              serverClients[i].print("M365 BLE ARRAY Screen"); 
            break;
          case ts_bms_array:
              serverClients[i].print("M365 BMS ARRAY Screen"); 
            break;
          case ts_x1_array:
              serverClients[i].print("M365 X1 ARRAY Screen"); 
            break;
          case ts_statistics:
              serverClients[i].print("M365 Statistics Screen");
            break;
        }
        //serverClients[i].print(ansiGRN); 
        //serverClients[i].print(ansiREDF);
        //serverClients[i].print("\033[?25l");
        //serverClients[i].printf(" %05d\r\n\r\n",tmpi++);
        //serverClients[i].print(ansiEND);
        //serverClients[i].print(ansiEND);
        tmpi++;
        if (tmpi % 2) {
          serverClients[i].println(" #\r\n");
        } else {
          serverClients[i].println("  \r\n");
        }
        
        switch (telnetscreen) {
          case ts_telemetrie:
              if (newdata) {
                serverClients[i].printf("\r\nBLE\r\n Throttle: %03d Brake %03d\r\n",bleparsed->throttle,bleparsed->brake);
                serverClients[i].printf("\r\nBMS Serial: xx Version: %05d\r\n", bmsparsed->fwversion);
                serverClients[i].printf(" Capacity Total: %05d Remaining %05d Percent %05d Temperature %05d Health %05d\r\n",bmsparsed->totalcapacity, bmsparsed->remainingcapacity, bmsparsed->remainingpercent, bmsparsed->temperature, bmsparsed->health);
                serverClients[i].printf(" Voltage: %05d Current %05d\r\n",bmsparsed->voltage, bmsparsed->current);
                serverClients[i].printf(" C1: %05d C2: %05d C3: %05d C4: %05d C5: %05d C6: %05d C7: %05d C8: %05d C9: %05d C10: %05d\r\n",bmsparsed->Cell1Voltage,bmsparsed->Cell2Voltage,bmsparsed->Cell3Voltage,bmsparsed->Cell4Voltage,bmsparsed->Cell5Voltage,bmsparsed->Cell6Voltage,bmsparsed->Cell7Voltage,bmsparsed->Cell8Voltage,bmsparsed->Cell9Voltage,bmsparsed->Cell10Voltage);
                serverClients[i].printf("\r\nESC Serial: xx Version: %05d", escparsed->fwversion);
                serverClients[i].printf(" Pin: 123456 Error %05d\r\n",escparsed->error);
                serverClients[i].printf(" Distance Total: %05d Trip: %05d Remaining %05d", escparsed->totaldistance,escparsed->tripdistance,escparsed->remainingdistance);
                serverClients[i].printf(" Trip Time: %05d\r\n",escparsed->triptime);
                serverClients[i].printf(" FrameTemp1: %05d FrameTemp2: %05d\r\n", escparsed->frametemp1, escparsed->frametemp2);
                serverClients[i].printf(" Speed: %05d Avg: %05d\r\n", escparsed->speed, escparsed->averagespeed);
                serverClients[i].printf(" Batt Percent: %05d\r\n",escparsed->battpercent);
                serverClients[i].printf(" Êcomode: %05d Kers: %05d Cruisemode: %05d Taillight: %05d\r\n", escparsed->ecomode, escparsed->kers, escparsed->cruisemode, escparsed->taillight);
                serverClients[i].printf("\r\nX1 Mode %03d LEDs %03d Light %03d BeepAction %03d\r\n",x1parsed->mode, x1parsed->battleds, x1parsed->light, x1parsed->beepaction);
              }
              telnetlastrefreshtimestamp=millis()+telnetrefreshanyscreen;
            break;
          case ts_statistics:
              serverClients[i].printf("Packets Received: %05d\r\nPackets CRC OK: %05d\r\nPackets CRC FAIL: %05d\r\n\r\nPackets per device Address:\r\n", packets_rec,packets_crcok,packets_crcfail);
              //for(i = 0; i < MAX_SRV_CLIENTS; i++){
              k = 0;
              for(uint16_t j = 0; j <=255; j++) {
                if (packetsperaddress[j]>=1) {
                  k++;
                  serverClients[i].printf("%02X -> %05d  ", j, packetsperaddress[j]);
                  if ((k % 5)==0) { serverClients[i].println(""); }
                }
              }
              //ServerClients[i].print 
              telnetlastrefreshtimestamp=millis()+telnetrefreshanyscreen;           
            break;
          case ts_esc_raw:
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    serverClients[i].printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      serverClients[i].print("- "); 
                    }
                  } //mod32
                  serverClients[i].printf("%02X ", escdata[j]);
              }
              telnetlastrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_ble_raw:
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    serverClients[i].printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      serverClients[i].print("- "); 
                    }
                  } //mod32
                  serverClients[i].printf("%02X ", bledata[j]);
              }
              telnetlastrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_bms_raw:
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    serverClients[i].printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      serverClients[i].print("- "); 
                    }
                  } //mod32
                  serverClients[i].printf("%02X ", bmsdata[j]);
              }
              telnetlastrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_x1_raw:
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    serverClients[i].printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      serverClients[i].print("- "); 
                    }
                  } //mod32
                  serverClients[i].printf("%02X ", x1data[j]);
              }
              telnetlastrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_esc_array:
          case ts_ble_array:
          case ts_bms_array:
          case ts_x1_array:
              serverClients[i].print("empty screen - not implemented");
              telnetlastrefreshtimestamp=millis()+telnetrefreshanyscreen;
              
            break;
        }
        yield();
    }
  } //i 0 to maxclients

}
void handle_telnet() {
  boolean hc = false;
   switch(telnetstate) {
    case telnetoff:
      break;
    case telnetturnon:
        server.begin();
        server.setNoDelay(true);
        Serial.print("### Telnetserver @ ");
        if (wlanstate==wlanap) {
          Serial.print(WiFi.softAPIP());  
        } else {
          Serial.print(WiFi.localIP());  
        }
        Serial.println(":23");
        telnetstate = telnetlistening;
        userconnecttimestamp = millis()+userconnecttimeout;  
      break;
    case telnetlistening: 
          //if (WiFi.isConnected()) {
            //check if there are any new clients
            if (server.hasClient()) {
              for(i = 0; i < MAX_SRV_CLIENTS; i++){
                //find free/disconnected spot
                if (!serverClients[i] || !serverClients[i].connected()){
                  if(serverClients[i]) serverClients[i].stop();
                  serverClients[i] = server.available();
                  if (!serverClients[i]) Serial.println("available broken");
                  Serial.print("### Telnet New client: ");
                  Serial.print(i); Serial.print(' ');
                  Serial.println(serverClients[i].remoteIP());
                  telnetstate = telnetclientconnected;
                  ledontime=250; ledofftime=250; ledcurrenttime = millis();
                  telnetlastrefreshtimestamp=millis()+telnetrefreshanyscreen;
                  break;
                }
              }
              if (i >= MAX_SRV_CLIENTS) {
                //no free/disconnected spot so reject
                server.available().stop();
                Serial.println("### Telnet rejected connection (max Clients)");
              }
            }
            if (userconnecttimestamp<millis()) {
              telnetstate = telnetturnoff;
            }
          //} //WiFi.isConnected
      break;
    case telnetclientconnected: 
        //TODO check if clients are still connected... else start client connect timer and fall back to listening mode
        hc = false;
        for(i = 0; i < MAX_SRV_CLIENTS; i++){
                  if (serverClients[i] && serverClients[i].connected()){
                      hc=true;
                  }
              } //i 0 to maxclients
        if (!hc) {
          //no more clients, restart listening timer....
          telnetstate = telnetturnon;
          Serial.println("### Telnet lost all clients - restarting telnet");
        } else {
          //still has clients, update telnet stuff
            if (telnetlastrefreshtimestamp<millis()) {
                  telnet_refreshscreen();
            } //telnetrefreshtimer

        } //else !hc
        //if (WiFi.isConnected()) {
          //check clients for data
          
          for(i = 0; i < MAX_SRV_CLIENTS; i++){
            if (serverClients[i] && serverClients[i].connected()){
              if(serverClients[i].available()){
                //get data from the telnet client and push it to the UART
                uint8_t tcmd = serverClients[i].read();
                Serial.printf("Telnet Command: %02X\r\n",tcmd);
                switch (tcmd) {
                  case 0x73: telnetscreen=ts_statistics; break; //s
                  case 0x74: telnetscreen=ts_telemetrie; break; //t
                  case 0x65: telnetscreen=ts_esc_raw; break; //e
                  case 0x62: telnetscreen=ts_bms_raw; break; //b
                  case 0x6E: telnetscreen=ts_ble_raw; break; //n
                  case 0x78: telnetscreen=ts_x1_raw; break; //x
                  case 0x45: telnetscreen=ts_esc_array; break; //E
                  case 0x42: telnetscreen=ts_bms_array; break; //B
                  case 0x4E: telnetscreen=ts_ble_array; break; //N
                  case 0x58: telnetscreen=ts_x1_array; break;  //X
                  case 0x72: reset_statistics(); break;  //r
                  

                } //switch
                //while(serverClients[i].available()) Serial1.write(serverClients[i].read());
              } //serverclients available
            } //if connected
          }  //for i
          
/*            else {
              if (serverClients[i]) {
                serverClients[i].stop();
              }
            }
          }
        //}
*/
      break;
    case telnetdisconnectclients:
        for(i = 0; i < MAX_SRV_CLIENTS; i++) {
          if (serverClients[i]) serverClients[i].stop();
        }
        telnetstate = telnetturnoff;

      break;
    case telnetturnoff:
        server.end();
        wlanstate=wlanturnoff; 
      break;
   } //switch (telnetstate)
  
} //handle_telnet

void WiFiEvent(WiFiEvent_t event) {
    //Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
      case SYSTEM_EVENT_WIFI_READY: Serial.println("### WifiEvent: SYSTEM_EVENT_WIFI_READY"); break; // < ESP32 WiFi ready
      case SYSTEM_EVENT_SCAN_DONE: Serial.println("### WifiEvent: SYSTEM_EVENT_SCAN_DONE"); break; // < ESP32 finish scanning AP
      case SYSTEM_EVENT_STA_START: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_START"); break; // < ESP32 station start
      case SYSTEM_EVENT_STA_STOP: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_STOP"); break; // < ESP32 station stop
      case SYSTEM_EVENT_STA_CONNECTED: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_CONNECTED"); break; // < ESP32 station connected to AP
      case SYSTEM_EVENT_STA_DISCONNECTED: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_DISCONNECTED"); break; // < ESP32 station disconnected from AP
      case SYSTEM_EVENT_STA_AUTHMODE_CHANGE: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_AUTHMODE_CHANGE"); break; // < the auth mode of AP connected by ESP32 station changed
      case SYSTEM_EVENT_STA_GOT_IP: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_GOT_IP"); break; // < ESP32 station got IP from connected AP
      case SYSTEM_EVENT_STA_LOST_IP: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_LOST_IP"); break; // < ESP32 station lost IP and the IP is reset to 0
      case SYSTEM_EVENT_STA_WPS_ER_SUCCESS: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_SUCCESS"); break; // < ESP32 station wps succeeds in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_FAILED: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_FAILED"); break; // < ESP32 station wps fails in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_TIMEOUT"); break; // < ESP32 station wps timeout in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_PIN: Serial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_PIN"); break; // < ESP32 station wps pin code in enrollee mode
      case SYSTEM_EVENT_AP_START: Serial.println("### WifiEvent: SYSTEM_EVENT_AP_START"); break; // < ESP32 soft-AP start
      case SYSTEM_EVENT_AP_STOP: Serial.println("### WifiEvent: SYSTEM_EVENT_AP_STOP"); break; // < ESP32 soft-AP stop
      case SYSTEM_EVENT_AP_STACONNECTED: apnumclientsconnected++; Serial.println("### WifiEvent: SYSTEM_EVENT_AP_STACONNECTED"); break; // < a station connected to ESP32 soft-AP
      case SYSTEM_EVENT_AP_STADISCONNECTED: apnumclientsconnected--; Serial.println("### WifiEvent: SYSTEM_EVENT_AP_STADISCONNECTED"); break; // < a station disconnected from ESP32 soft-AP
      case SYSTEM_EVENT_AP_PROBEREQRECVED: Serial.println("### WifiEvent: SYSTEM_EVENT_AP_PROBEREQRECVED"); break; // < Receive probe request packet in soft-AP interface
      case SYSTEM_EVENT_GOT_IP6: Serial.println("### WifiEvent: SYSTEM_EVENT_GOT_IP6"); break; // < ESP32 station or ap or ethernet interface v6IP addr is preferred
      case SYSTEM_EVENT_ETH_START: Serial.println("### WifiEvent: SYSTEM_EVENT_ETH_START"); break; // < ESP32 ethernet start
      case SYSTEM_EVENT_ETH_STOP: Serial.println("### WifiEvent: SYSTEM_EVENT_ETH_STOP"); break; // < ESP32 ethernet stop
      case SYSTEM_EVENT_ETH_CONNECTED: Serial.println("### WifiEvent: SYSTEM_EVENT_ETH_CONNECTED"); break; // < ESP32 ethernet phy link up
      case SYSTEM_EVENT_ETH_DISCONNECTED: Serial.println("### WifiEvent: SYSTEM_EVENT_ETH_DISCONNECTED"); break; // < ESP32 ethernet phy link down
      case SYSTEM_EVENT_ETH_GOT_IP: Serial.println("### WifiEvent: SYSTEM_EVENT_ETH_GOT_IP"); break; // < ESP32 ethernet got IP from connected AP
      case SYSTEM_EVENT_MAX: Serial.println("### WifiEvent: SYSTEM_EVENT_MAX"); break; //
    }
} //WiFiEvent

void handle_wlan() {
  switch(wlanstate) {
    case wlanoff: 
      break;
    case wlanturnapon:
        WiFi.disconnect(true);
        delay(500);
        WiFi.mode(WIFI_OFF);
        yield();
        WiFi.mode(WIFI_AP);
        yield();
        apnumclientsconnected=0;
        apnumclientsconnectedlast=0;
        WiFi.onEvent(WiFiEvent);
        WiFi.softAP(apssid, appassword);
        yield();
        //IPAddress myIP = WiFi.softAPIP();
        Serial.printf("### WLAN AP\r\nSSID: %s, AP IP address: ", apssid);  Serial.println( WiFi.softAPIP());
        display.fillRect(0,56,display.width(), 8, BLACK);
        display.setCursor(0,56);
        display.print("AP Mode");
        display.fillRect(0,40,display.width(), 8, BLACK);
        display.setCursor(0,40);
        display.print( WiFi.softAPIP());
        display.display();
        if (telnetstate==telnetoff) {
          telnetstate = telnetturnon;
        }
        wlanconnecttimestamp = millis()+wlanapconnecttimeout;            
        wlanstate = wlanap;
        ArduinoOTA.begin();
        //start OTA
      break;
    case wlanturnstaon:
        Serial.println("### Searching for known wlan");
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        #ifdef staticip
          if (!WiFi.config(ip, gateway, subnet, dns)) {
            Serial.println("### STA Failed to configure");
          }
        #endif
        currentssidindex = 0;
        WiFi.onEvent(WiFiEvent);
        WiFi.begin(ssid1, password1);
        wlanconnecttimestamp = millis()+wlanconnecttimeout;
        display.fillRect(0,56,display.width(), 8, BLACK);
        display.setCursor(0,56);
        display.printf("W %d",currentssidindex);
        display.display();  
        Serial.printf("###  WLan searching for ssidindex %d\r\n",currentssidindex);
        wlanstate = wlansearching;
        ledontime=50; ledofftime=450; ledcurrenttime = millis();
        //start wlan connect timeout
      break;
    case wlansearching:
        if (WiFi.status() != WL_CONNECTED) {
          if (wlanconnecttimestamp<millis()) {
            //timeout, test next wlan
            currentssidindex++;
            if (currentssidindex<maxssids) {
              if (currentssidindex==1) { WiFi.begin(ssid2, password2); }
              if (currentssidindex==2) { WiFi.begin(ssid3, password3); }
              wlanconnecttimestamp = millis()+wlanconnecttimeout;
              display.fillRect(0,56,display.width(), 8, BLACK);
              display.setCursor(0,56);
              display.printf("W %d",currentssidindex);
              display.display();
              Serial.printf("### WLan searching for ssidindex %d\r\n",currentssidindex);
            } else {
              Serial.println("### WLAN search failed, starting Access Point");
              wlanstate = wlanturnapon;
            }
          }
        } else {
          wlanstate = wlanconnected;
          Serial.print("### WLAN Connected to ");
          Serial.print(WiFi.SSID());
          Serial.print(", IP is ");
          Serial.println(WiFi.localIP());
          display.fillRect(0,56,display.width(), 8, BLACK);
          display.setCursor(0,56);
          display.print("WC ");
          display.print(WiFi.SSID());
          display.fillRect(0,40,display.width(), 8, BLACK);
          display.setCursor(0,40);
          display.print(WiFi.localIP());
          display.display();
          if (telnetstate==telnetoff) {
                telnetstate = telnetturnon;
            }
          ledontime=500; ledofftime=500; ledcurrenttime = millis();
          ArduinoOTA.begin();
        }

      break;
    case wlanconnected:
          if (WiFi.status() != WL_CONNECTED) {
              Serial.println("### WiFi lost connection!");
              display.fillRect(0,56,display.width(), 8, BLACK);
              display.setCursor(0,56);
              display.print("W CONN LOST");
              display.print(WiFi.SSID());
              display.display();
              if (telnetstate!=telnetoff) {
                  telnetstate = telnetdisconnectclients;
              }
             wlanstate = wlanturnoff;

          }
      break;
    case wlanap: //make 2 states - waiting for clients & has clients
        //without wifi events: apnumclientsconnected=WiFi.softAPgetStationNum();
        if (apnumclientsconnected!=apnumclientsconnectedlast) {
          //num of connect clients changed
          apnumclientsconnectedlast = apnumclientsconnected;
          Serial.printf("### AP Clients connected changed: %d -> %d\r\n",apnumclientsconnectedlast, apnumclientsconnected);
          display.fillRect(0,56,display.width(), 8, BLACK);
          display.setCursor(0,56);
          display.printf("AP Clients %d",apnumclientsconnected);
          display.display();
          if (apnumclientsconnected==0) {
              //no one connnected, but there was someone connected.... restart timeout
              wlanconnecttimestamp = millis()+wlanapconnecttimeout;
          } //if (apnumclientsconnected==0) 
        } //if (apnumclientsconnected!=apnumclientsconnectedlast)
        yield();
        if (apnumclientsconnected==0) {
          if (wlanconnecttimestamp<millis()) {
            wlanstate=wlanturnoff; 
          } //if (wlanconnecttimestamp<millis())
        } //if (apnumclientsconnected==0)
      break;
    case wlanturnoff:
#ifdef usemqtt
        if (client.connected) {
          client.disconnect();
        }        
#endif
        ArduinoOTA.end();
        WiFi.softAPdisconnect();
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        Serial.println("### WIFI OFF");
        display.fillRect(0,56,display.width(), 8, BLACK);
        display.setCursor(0,56);
        display.print("WLAN OFF");
        display.fillRect(0,40,display.width(), 8, BLACK);
        display.display();
        wlanstate=wlanoff; 
        //STOP OTA    
      break;
  } //switch wlanstate
} //handle_wlan


#ifdef debug_dump_states
  void print_states() {
    if (wlanstate!=wlanstateold) {
      Serial.printf("### WLANSTATE %d -> %d\r\n",wlanstateold,wlanstate);
      wlanstateold=wlanstate;
    }
    if (telnetstate!=telnetstateold) {
      Serial.printf("### TELNETSTATE %d -> %d\r\n",telnetstateold,telnetstate);
      telnetstateold=telnetstate;
    }
    if (m365receiverstate!=m365receiverstateold) {
      Serial.printf("### M365RecState: %d -> %d\r\n",m365receiverstateold,m365receiverstate);
      m365receiverstateold=m365receiverstate;
    }
    if (m365packetstate!=m365packetstateold) {
      Serial.printf("M365PacketState: %d -> %d\r\n",m365packetstateold,m365packetstate);
      m365packetstateold=m365packetstate;
    }    
  }
#endif



void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, sdapin,sclpin);
  display.clearDisplay();
  display.drawPixel(10, 10, WHITE); 
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(swversion);
  /*display.setCursor(0,line6);
  display.println("3");
  display.setCursor(8,line6);
  display.println("5");
  display.setCursor(16,line6);
  display.println("8");
*/
  //display.setTextColor(BLACK, WHITE); // 'inverted' text
  //display.println(3.141592);
  //display.setTextSize(2);
  //display.setTextColor(WHITE);
  //display.print("0x"); display.println(0xDEADBEEF, HEX);
  display.display();
  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);
  Serial.begin(115200);
  Serial.println(swversion);
  wlanstate=wlanturnstaon;
  uint64_t cit;
  #ifdef ESP32
    cit = ESP.getEfuseMac();
    sprintf(chipid,"%02X%02X%02X",(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    Serial.println(chipid);
    sprintf(mac,"%02X%02X%02X%02X%02X%02X",(uint8_t)(cit),(uint8_t)(cit>>8),(uint8_t)(cit>>16),(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    /*
    Serial.printf("ESP32 Chip ID_1 = %04X\r\n",(uint16_t)(cit>>48));
    Serial.printf("ESP32 Chip ID_2 = %04X\r\n",(uint16_t)(cit>>32));
    Serial.printf("ESP32 Chip ID_3 = %04X\r\n",(uint16_t)(cit>>16));
    Serial.printf("ESP32 Chip ID_4 = %04X\r\n",(uint16_t)(cit));
    Serial.printf("ESP32 Chip ID_A = %08X\r\n",(uint32_t)(cit>>32));
    Serial.printf("ESP32 Chip ID_B = %08X\r\n",(uint32_t)(cit));
    Serial.printf("ESP32 Chip ID_X = %08X%08X\r\n",(uint32_t)(cit>>32),(uint32_t)(cit));
    Serial.printf("ESP32 Chip ID_Y = %16X\r\n",cit);
    */
  #elif defined(ESP8266)
    cit = ESP.getChipId();
    sprintf(chipid,"%02X%02X%02X",(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    byte mc[6];
    WiFi.macAddress(mc);
    sprintf(mac, "%02X%02X%02X%02X%02X%02X", mc[0], mc[1], mc[2], mc[3], mc[4], mc[5]);
  #endif
  start_m365();
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAStart", mqtt_clientID);
      client.publish(tmp1, "OTA Starting");
    }
#endif
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C, sdapin,sclpin);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("OTA Start");
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAEND", mqtt_clientID);
      client.publish(tmp1, "OTA Done");
    }
#endif
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("OTA End");
    display.display();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA %u%%\r", (progress / (total / 100)));
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.printf("OTA %u%%", (progress / (total / 100)));
    display.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (error == OTA_AUTH_ERROR) sprintf(tmp1,"%s","Auth Failed");
    else if (error == OTA_BEGIN_ERROR) sprintf(tmp1,"%s","Begin Failed");
    else if (error == OTA_CONNECT_ERROR) sprintf(tmp1,"%s","Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) sprintf(tmp1,"%s","Receive Failed");
    else if (error == OTA_END_ERROR) sprintf(tmp1,"%s","End Failed");
    sprintf(tmp2, "OTA Error [%u]: %s", error, tmp1);
    Serial.println(tmp2);
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAError", mqtt_clientID);
      client.publish(tmp1, tmp2);
    }
#endif
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.printf("OTA ERROR: %u %s", error,tmp1);
    display.display();
    delay(2000);

  });
  ArduinoOTA.setPassword(OTApwd);
  sprintf(tmp2, "es-m36-%s",mac);
  ArduinoOTA.setHostname(tmp2);
} //setup

void handle_led() {
  if (millis()>ledcurrenttime) {
    if (digitalRead(led)) {
      digitalWrite(led,LOW);
      ledcurrenttime = millis()+ledontime;
    } else {
      digitalWrite(led,HIGH);
      ledcurrenttime = millis()+ledofftime;
    }
  }
}

void loop() {
  handle_wlan();
  handle_telnet();
  m365_receiver(); 
  m365_handlepacket();
  #ifdef debug_dump_states
    print_states();
  #endif
  //m365_detectapp(); //detect if smartphone is connected and requests data, so we stay quiet
  //m365_transmitter
  handle_led();
#ifdef usemqtt
  client.loop();
#endif
  ArduinoOTA.handle();
}

