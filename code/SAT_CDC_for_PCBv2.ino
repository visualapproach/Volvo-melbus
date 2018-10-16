/*
  New in this version:
    Hardwarewise - Adjusted pin numbers to match archi's PCB v2.0
    Softwarewise - Changed the way RGB LEDs are managed. Press "3" on HU to cycle through colors on driver side.
                    press "4" to do the same on passenger side. See github for more info.
                   Changed customText variable to hold four lines of text.
                   Battery voltage now only show one decimal. It's an estimate anyway.
                   Steering wheel "next/prev" is now working. (Same as pressing 2/6)
                   Switching to/from SAT source toggles a pin (A6). Can be used to turn of devices like the BT module. (github request)
                   (Not tested by me yet)

  Todo (someone): if possible, enable scrolling text. This should be possible since the text messages is larger than the screen.
  I guess scrolling can be turned on by
  1) editing some bits in the init messages
  2) editing some bits in the text headers
  3) some specific character combination in the text itself
  (Text IS scrolling twice after pressing source/sat display/ and choosing title/artist etc on the HU)

  By Thomas Landahl, 2018-10-16
  https://github.com/visualapproach/Volvo-melbus
  
  Comm sniffing and some code contribution by Vincent Gijsen. Thanks a LOT!

*/

#include <EEPROM.h>

#define INT_NUM (byte)0         //Interrupt number (0/1 on ATMega 328P)
#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)3     //Pin D3  - Data
#define MELBUS_BUSY (byte)4     //Pin D4  - Busy

const byte mutePin = 9;
const byte playPin = 10;
const byte prevPin = 11;
const byte nextPin = 12;
const byte upPin = 7;    //volume up (not used on PCB)
const byte downPin = 8;  //volume down (not used on PCB)

const byte LED_RED_LFT = 14;  //A0
const byte LED_GRN_LFT = 15;  //A1
const byte LED_BLU_LFT = 16;  //A2
const byte LED_RED_RGT = 17;  //A3
const byte LED_GRN_RGT = 18;  //A4
const byte LED_BLU_RGT = 19;  //A5
const byte MISC = 20;         //A6
const byte BATTERY = 21;      //A7 - PCB uses this

byte lftClr;
byte rgtClr;
String colors[8] = {
  "Black ", "Blue  ", "Green ", "Cyan  ", "Red   ", "Purple", "Yellow", "White "
};

const int R_CAR = 10; //cable resistance from battery to device
const int R_1 = 3300; //r1 of voltage divider (connected to GND)
const int R_2 = 6800; //r2 of voltage divider (connected to RAW)
const float T_LOSS = 0.8; //if circuit is protected with a transistor and diode, put total voltage drop here.

byte track = 0x01; //Display show HEX value, not DEC. (A-F not "allowed")
byte cd = 0x01; //1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)


//volatile variables used inside AND outside of ISP
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;


byte byteToSend = 0;          //global to avoid unnecessary overhead
bool reqMasterFlag = false;   //set this to request master mode (and sendtext) at a proper time.

#define RESPONSE_ID 0xC5      //ID while responding to init requests (which will use base_id)
#define BASE_ID 0xC0          //ID when getting commands from HU
#define MASTER_ID 0xC7

#define CDC_RESPONSE_ID 0xEE  //ID while responding to init requests (which will use base_id)
#define CDC_MASTER_ID 0xEF    //ID while requesting/beeing master
#define CDC_BASE_ID 0xE8      //ID when getting commands from HU
#define CDC_ALT_ID 0xE9       //Alternative ID when getting commands from HU


byte textHeader[] = {0xFC, 0xC6, 0x73, 0x01};
byte textRow = 2;
byte customText[4][36] = {
  {""},
  {"visualapproach"},
  {""},
  {""}
};

//HU asks for line 3 and 4 below at startup. They can be overwritten by customText if you change textRow to 3 or 4
byte textLine[4][36] = {
  {"Line 1"},           //is overwritten by init-sequence ("Volvo!")
  {"Line 2"},           //is overwritten by customText[1][]
  {"3=cycle L col"},    //changes if pressing 3-button
  {"4=cycle R col"}     //changes if pressing 4-button
};

const byte C1_Init_1[9] = {
  0x10, 0x00, 0xc3, 0x01,
  0x00, 0x81, 0x01, 0xff,
  0x00
};
const byte SO_C1_Init_1 = 9;

const byte C1_Init_2[11] = {
  0x10, 0x01, 0x81,
  'V', 'o', 'l', 'v', 'o', '!', ' ', ' '
};
const byte SO_C1_Init_2 = 11;


const byte C2_Init_1[] = {
  0x10, 0x01, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff
};
const byte SO_C2_Init_1 = 19;


const byte C3_Init_0[30] = {
  0x10, 0x00, 0xfe, 0xff,
  0xff, 0xdf, 0x3f, 0x29,
  0x2c, 0xf0, 0xde, 0x2f,
  0x61, 0xf4, 0xf4, 0xdf,
  0xdd, 0xbf, 0xff, 0xbe,
  0xff, 0xff, 0x03, 0x00,
  0xe0, 0x05, 0x40, 0x00,
  0x00, 0x00
} ;
const byte SO_C3_Init_0 = 30;

const byte C3_Init_1[30] = {
  0x10, 0x01, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff
};
const byte SO_C3_Init_1 = 30;

const byte C3_Init_2[30] = {
  0x10, 0x02, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff
};
const byte SO_C3_Init_2 = 30;




//Defining the commands. First byte is the length of the command.
#define MRB_1 {3, 0x00, 0x1C, 0xEC}            //Master Request Broadcast version 1
#define MRB_2 {3, 0x00, 0x1E, 0xEC}            //Master Request Broadcast version 2 (maybe this is second init seq?)
#define MI {3, 0x07, 0x1A, 0xEE}               //Main init sequence
#define SI {3, 0x00, 0x1E, 0xED}               //Secondary init sequence (turn off ignition, then on)
//changed from 00 1D ED

#define C1_1 {5, 0xC1, 0x1B, 0x7F, 0x01, 0x08} //Respond with c1_init_1
#define C1_2 {5, 0xC1, 0x1D, 0x73, 0x01, 0x81} //Respond with c1_init_2 (text)

#define C2_0 {4, 0xC2, 0x1D, 0x73, 0x00}       //Get next byte (nn) and respond with 10, 0, nn, 0,0 and 14 * 0x20 (possibly text)
#define C2_1 {4, 0xC2, 0x1D, 0x73, 0x01}       //Same as above? Answer 19 bytes (unknown)

#define C3_0 {4, 0xC3, 0x1F, 0x7C, 0x00}       //Respond with c1_init_2 (text)
#define C3_1 {4, 0xC3, 0x1F, 0x7C, 0x01}       //Respond with c1_init_2 (text)
#define C3_2 {4, 0xC3, 0x1F, 0x7C, 0x02}       //Respond with c1_init_2 (text)

#define C5_1 {3, 0xC5, 0x19, 0x73}  //C5, 19, 73, xx, yy. Answer 0x10, xx, yy + free text. End with 00 00 and pad with spaces

#define CMD_1 {3, 0xC0, 0x1B, 0x76}            //Followed by: [00, 92, FF], OR [01, 03 ,FF] OR [02, 05, FF]. Answer 0x10
#define PUP {4, 0xC0, 0x1C, 0x70, 0x02}      //Wait 2 bytes and answer 0x90?
#define CMD_3 {5, 0xC0, 0x1D, 0x76, 0x80, 0x00} //Answer: 0x10, 0x80, 0x92
#define PWR_OFF {6,0xC0, 0x1C, 0x70, 0x00, 0x80, 0x01} //answer one byte
#define PWR_SBY {6,0xC0, 0x1C, 0x70, 0x01, 0x80, 0x01} //answer one byte
#define IGN_OFF {3, 0x00, 0x18, 0x12}           //this is the last message before HU goes to Nirvana

#define BTN {4, 0xC0, 0x1D, 0x77, 0x81}        //Read next byte which is the button #. Respond with 3 bytes
#define NXT {5, 0xC0, 0x1B, 0x71, 0x80, 0x00}  //Answer 1 byte
#define PRV {5, 0xC0, 0x1B, 0x71, 0x00, 0x00}  //Answer 1 byte
#define SCN {4, 0xC0, 0x1A, 0x74, 0x2A}        //Answer 1 byte

#define CDC_CIR {3, CDC_BASE_ID, 0x1E, 0xEF}             //Cartridge info request. Respond with 6 bytes
#define CDC_TIR {5, CDC_ALT_ID, 0x1B, 0xE0, 0x01, 0x08}  //track info req. resp 9 bytes
#define CDC_NXT {5, CDC_BASE_ID, 0x1B, 0x2D, 0x40, 0x01} //next track.
#define CDC_PRV {5, CDC_BASE_ID, 0x1B, 0x2D, 0x00, 0x01} //prev track
#define CDC_CHG {3, CDC_BASE_ID, 0x1A, 0x50}             //change cd
#define CDC_PUP {3, CDC_BASE_ID, 0x19, 0x2F}             //power up. resp ack (0x00).
#define CDC_PDN {3, CDC_BASE_ID, 0x19, 0x22}             //power down. ack (0x00)
#define CDC_FFW {3, CDC_BASE_ID, 0x19, 0x29}             //FFW. ack
#define CDC_FRW {3, CDC_BASE_ID, 0x19, 0x26}             //FRW. ack
#define CDC_SCN {3, CDC_BASE_ID, 0x19, 0x2E}             //scan mode. ack
#define CDC_RND {3, CDC_BASE_ID, 0x19, 0x52}             //random mode. ack
#define CDC_NU {3, CDC_BASE_ID, 0x1A, 0x50}              //not used
//#define CDC_MI {0x07, 0x1A, 0xEE},         //main init seq. wait for BASE_ID and respond with RESPONSE_ID.
//#define CDC_SI {0x00, 0x1C, 0xED},         //secondary init req. wait for BASE_ID and respond with RESPONSE_ID.
//#define CDC_MRB {0x00, 0x1C, 0xEC}         //master req broadcast. wait for MASTER_ID and respond with MASTER_ID.

//This list can be quite long. We have approx 700 us between the received bytes.
const byte commands[][7] = {
  MRB_1,  // 0 now we are master and can send stuff (like text) to the display!
  MI,     // 1 main init
  SI,     // 2 sec init (00 1E ED respond 0xC5 !!)
  CMD_1,  // 3 follows: [0, 92, FF], OR [1,3 ,FF] OR [2, 5 FF]
  PUP,  // 4 wait 2 bytes and answer 0x90?
  MRB_2,  // 5 alternative master req bc
  CMD_3,  // 6 unknown. Answer: 0x10, 0x80, 0x92
  C1_1,   // 7 respond with c1_init_1
  C1_2,   // 8 respond with c1_init_2 (contains text)
  C3_0,   // 9 respond with c3_init_0
  C3_1,   // 10 respond with c3_init_1
  C3_2,   // 11 respond with c3_init_2
  C2_0,   // 12 get next byte (nn) and respond with 10, 0, nn, 0,0 and 14 of 0x20
  C2_1,   // 13
  C5_1,   // 14
  BTN,    // 15
  NXT,    // 16
  PRV,    // 17
  SCN,    // 18
  PWR_OFF,// 19
  PWR_SBY,// 20
  IGN_OFF, // 21
  CDC_CIR, // 22
  CDC_TIR, // 23
  CDC_NXT, // 24
  CDC_PRV, // 25
  CDC_CHG, // 26
  CDC_PUP, // 27
  CDC_PDN, // 28
  CDC_FFW, // 29
  CDC_FRW, // 30
  CDC_SCN, // 31
  CDC_RND, // 32
  CDC_NU   // 33
};

const byte listLen = 34; //how many rows in the above array

// some CDC (CD-CHANGER) data
byte trackInfo[] = {0x00, 0x02, 0x00, cd, 0x80, track, 0xC7, 0x0A, 0x02}; //9 bytes
byte startByte = 0x08; //on powerup - change trackInfo[1] & [8] to this
byte stopByte = 0x02; //same on powerdown
byte cartridgeInfo[] = {0x00, 0xFC, 0xFF, 0x4A, 0xFC, 0xFF};




/*
 *      **** SETUP ****
*/


void setup() {
  //Disable timer0 interrupt. It's is only bogging down the system. We need speed!
  TIMSK0 &= ~_BV(TOIE0);

  //All lines are idle HIGH
  pinMode(MELBUS_DATA, INPUT_PULLUP);
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  pinMode(nextPin, OUTPUT);
  pinMode(prevPin, OUTPUT);
  pinMode(playPin, OUTPUT);
  digitalWrite(nextPin, LOW);
  digitalWrite(playPin, LOW);
  digitalWrite(prevPin, LOW);

  //set analog pins A0 through A7 to output/off
  for (byte i = 14; i < 22; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  pinMode(BATTERY, INPUT);

  //Initiate serial communication to debug via serial-usb (arduino)
  //Better off without it.
  //Serial printing takes a lot of time!!
  Serial.begin(115200);
  Serial.println("Calling HU");

  recall(); //get stored states from EEPROM and output to the LED driver.

  //Activate interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);

  //Call function that tells HU that we want to register a new device
  melbusInitReq();
}




/*        **************************************************

          MAIN LOOP

          **************************************************
*/


void loop() {
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static long runOnce = 300000;     //counts down on every received message from HU. Triggers when it is passing 1.
  static long runPeriodically = 100000; //same as runOnce but resets after each countdown.
  static bool powerOn = true;
  static long HWTicks = 0;      //age since last BUSY switch
  static long ComTicks = 0;     //age since last received byte
  static long ConnTicks = 0;    //age since last message to SIRIUS SAT
  static long timeout = 1000000; //should be around 10-20 secs
  static byte matching[listLen];     //Keep track of every matching byte in the commands array

  //these variables are reset every loop
  byte byteCounter = 1;  //keep track of how many bytes is sent in current command
  byte melbus_log[99];  //main init sequence is 61 bytes long...
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  HWTicks++;
  if (powerOn) {
    ComTicks++;
    ConnTicks++;
  } else {
    ComTicks = 1;
    ConnTicks = 1;
    //to avoid a lot of serial.prints when its 0
  }

  //check BUSY line active (active low)
  while (!BUSY) {
    HWTicks = 0;  //reset age

    //Transmission handling here...
    if (byteIsRead) {
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      ComTicks = 0; //reset age
      melbus_log[byteCounter - 1] = lastByte;
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < listLen; cmd++) {
        //check if this byte is matching
        if (lastByte == commands[cmd][byteCounter]) {
          matching[cmd]++;
          //check if a complete command is received, and take appropriate action
          if ((matching[cmd] == commands[cmd][0]) && (byteCounter == commands[cmd][0])) {
            byte cnt = 0;
            byte b1 = 0, b2 = 0;
            ConnTicks = 0;  //reset age

            switch (cmd) {

              //0, MRB_1
              case 0:
                //wait for master_id and respond with same
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      SendText(textRow);
                      //Serial.println("MRB1");
                      break;
                    }
                  }
                }
                //Serial.println("MRB 1");
                break;

              //1, MAIN INIT
              case 1:
                //wait for base_id and respond with response_id
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                    }
                    if (melbus_ReceivedByte == CDC_BASE_ID) {
                      byteToSend = CDC_RESPONSE_ID;
                      SendByteToMelbus();
                    }
                  }
                }
                //Serial.println("main init");
                break;

              //2, Secondary INIT
              case 2:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == CDC_BASE_ID) {
                      byteToSend = CDC_RESPONSE_ID;
                      SendByteToMelbus();
                    }
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                    }
                  }
                }
                //Serial.println("2nd init");
                break;

              //CMD_1. answer 0x10
              case 3:
                // we read 3 different tuple bytes (0x00 92), (01,3) and (02,5), response is always 0x10;
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    cnt++;
                  }
                  if (cnt == 2) {
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    break;
                  }
                }
                powerOn = true;
                //Serial.println("Cmd 1");
                break;


              //PUP. power on?
              case 4:
                // {0xC0, 0x1C, 0x70, 0x02} we respond 0x90;
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    cnt++;
                  }
                  if (cnt == 2) {
                    byteToSend = 0x90;
                    SendByteToMelbus();
                    break;
                  }
                }
                //Serial.println("power on");
                digitalWrite(MISC, HIGH);
                break;

              //MRB_2
              case 5:
                // {00 1E EC };
                //wait for master_id and respond with same
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      SendText(textRow);
                      //Serial.println("MRB2");
                      break;
                    }
                  }
                }
                //Serial.println("MRB 2");
                break;

              // CMD_3,  // 6 unknown. Answer: 0x10, 0x80, 0x92
              case 6:
                byteToSend = 0x10;
                SendByteToMelbus();
                byteToSend = 0x80;
                SendByteToMelbus();
                byteToSend = 0x92;
                SendByteToMelbus();
                //Serial.println("cmd 3");
                break;

              //C1_1,    7 respond with c1_init_1
              case 7:
                for (byte i = 0; i < SO_C1_Init_1; i++) {
                  byteToSend = C1_Init_1[i];
                  SendByteToMelbus();
                }
                //Serial.println("C1 1");
                break;

              //C1_2,   8 respond with c1_init_2 (contains text)
              case 8:
                for (byte i = 0; i < SO_C1_Init_2; i++) {
                  byteToSend = C1_Init_2[i];
                  SendByteToMelbus();
                }
                //Serial.println("C1_2");
                break;

              //  C3_0,    9 respond with c3_init_0
              case 9:
                for (byte i = 0; i < SO_C3_Init_0; i++) {
                  byteToSend = C3_Init_0[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 0");
                break;

              //C3_1,    10 respond with c3_init_1
              case 10:
                for (byte i = 0; i < SO_C3_Init_1; i++) {
                  byteToSend = C3_Init_1[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 1");
                break;

              //C3_2,   11 respond with c3_init_2
              case 11:
                for (byte i = 0; i < SO_C3_Init_2; i++) {
                  byteToSend = C3_Init_2[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 2");
                break;

              //   C2_0,    12 get next byte (nn) and respond with 10, 0, nn, 0,0 and (14 times 0x20)
              // possibly a text field?
              case 12:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = melbus_ReceivedByte;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x20;
                    for (byte b = 0; b < 14; b++) {
                      SendByteToMelbus();
                    }
                    break;
                  }
                }
                //Serial.print("C2_0");
                //Serial.println(melbus_ReceivedByte, HEX);
                break;

              //C2_1,    13 respond as 12
              case 13:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    byteToSend = 0x01;
                    SendByteToMelbus();
                    byteToSend = melbus_ReceivedByte;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x20;
                    for (byte b = 0; b < 14; b++) {
                      SendByteToMelbus();
                    }
                    break;
                  }
                }
                //Serial.print("C2_1");
                //Serial.println(melbus_ReceivedByte, HEX);
                break;

              //C5_1
              case 14:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b1 = melbus_ReceivedByte;
                    break;
                  }
                }
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b2 = melbus_ReceivedByte;
                    break;
                  }
                }
                byteToSend = 0x10;
                SendByteToMelbus();
                byteToSend = b1;
                SendByteToMelbus();
                byteToSend = b2;
                SendByteToMelbus();
                for (byte b = 0; b < 36; b++) {
                  byteToSend = textLine[b2 - 1][b];
                  SendByteToMelbus();
                }
                //Serial.print("C5_1");
                break;

              //BTN
              case 15:
                //wait for next byte to get CD number
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b1 = melbus_ReceivedByte;
                    break;
                  }
                }
                byteToSend = 255; //0x00;  //no idea what to answer
                //but HU is displaying "no artist" for a short while when sending zeroes.
                //Guessing there should be a number here indicating that text will be sent soon.
                SendByteToMelbus();
                byteToSend = 255; //0x00;  //no idea what to answer
                SendByteToMelbus();
                byteToSend = 255; //0x00;  //no idea what to answer
                SendByteToMelbus();

                switch (b1) {
                  //0x1 to 0x6 corresponds to cd buttons 1 to 6 on the HU (650) (SAT 1)
                  //7-13 on SAT 2, and 14-20 on SAT 3
                  //button 1 is always sent (once) when switching to SAT1.
                  case 0x1:
                    //toggleOutput(LEDMISC1); //turn on/off one output pin.
                    //Not used since it will be triggered by setting SAT1
                    break;
                  case 0x2:
                    nextTrack();  //unfortunately the steering wheel button equals btn #2
                    break;
                  case 0x3:
                    cycleLeft(); //change LED color
                    break;
                  case 0x4:
                    cycleRight(); //change LED color
                    break;
                  case 0x5:
                    //not used for the moment
                    break;
                  case 0x6:   //unfortunately the steering wheel button equals btn #6
                    prevTrack();
                    break;
                }
                //Serial.print("you pressed CD #");
                //Serial.println(b1);
                break;

              //NXT
              case 16:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                nextTrack();
                //Serial.println("NXT");
                break;

              //PRV
              case 17:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                prevTrack();
                //Serial.println("PRV");
                break;

              //SCN
              case 18:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                play();
                //Serial.println("SCN");
                break;

              //PWR OFF
              case 19:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                //Serial.println("Power down");
                powerOn = false;
                digitalWrite(MISC, LOW);
                break;

              //PWR SBY
              case 20:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                //Serial.println("Power stby");
                powerOn = false;
                break;

              //IGN_OFF
              case 21:
                //Serial.println("Ignition OFF");
                powerOn = false;
                break;

              //
              case 22:
                SendCartridgeInfo();
                break;

              //
              case 23:
                SendTrackInfo();
                break;

              //
              case 24:
                track++;
                fixTrack();
                trackInfo[5] = track;
                nextTrack();
                break;

              //
              case 25:
                track--;
                fixTrack();
                trackInfo[5] = track;
                prevTrack();
                break;

              //
              case 26:
                changeCD();
                break;

              //CDC_PUP
              case 27:
                byteToSend = 0x00;
                SendByteToMelbus();
                trackInfo[1] = startByte;
                trackInfo[8] = startByte;
                digitalWrite(MISC, HIGH);
                break;

              //CDC_PDN
              case 28:
                byteToSend = 0x00;
                SendByteToMelbus();
                trackInfo[1] = stopByte;
                trackInfo[8] = stopByte;
                digitalWrite(MISC, LOW);
                break;

              //CDC_FFW
              case 29:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_FRW
              case 30:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_SCN
              case 31:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_RND
              case 32:
                byteToSend = 0x00;
                SendByteToMelbus();
                play();
                break;

              //CDC_NU
              case 33:

                break;

            } //end switch
            break;    //bail for loop. (Not meaningful to search more commands if one is already found)
          } //end if command found
        } //end if lastbyte matches
      }  //end for cmd loop
      byteCounter++;
    }  //end if byteisread
    //Update status of BUSY line, so we don't end up in an infinite while-loop.
    BUSY = PIND & (1 << MELBUS_BUSY);
  }


  //Do other stuff here if you want. MELBUS lines are free now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.

  //Printing transmission log (from HU, excluding our answer and things after it)
  //if (ComTicks == 0) {                    //print all messages
  if (ComTicks == 0 && ConnTicks != 0) {    //print unmatched messages (unknown)
    for (byte b = 0; b < byteCounter - 1; b++) {
      Serial.print(melbus_log[b], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  //runOnce is counting down to zero and stays there
  //after that, runPeriodically is counting down over and over...
  if (runOnce >= 1) {
    runOnce--;
  } else if (runPeriodically > 0) runPeriodically--;


  //check if BUSY-line is alive
  if (HWTicks > timeout) {
    Serial.println("BUSY line problem");
    HWTicks = 0;
    ComTicks = 0;     //to avoid several init requests at once
    ConnTicks = 0;    //to avoid several init requests at once
    //while (1);      //maybe do a reset here, after a delay.
    melbusInitReq();  //worth a try...
  }

  //check if we are receiving any data
  if (ComTicks > timeout) {
    Serial.println("COM failure (check CLK line)");
    ComTicks = 0;
    ConnTicks = 0;    //to avoid several init requests at once
    melbusInitReq();  //what else to do...
  }

  //check if HU is talking to us specifically, otherwise force it.
  if (ConnTicks > timeout) {
    Serial.println("Lost connection. Re-initializing");
    ConnTicks = 0;
    melbusInitReq();
  }


  //Reset stuff
  melbus_Bitposition = 7;
  for (byte i = 0; i < listLen; i++) {
    matching[i] = 0;
  }

  //initiate MRB2 to send text (from one of the four textlines in customText[textRow][])
  if ((runOnce == 1) || reqMasterFlag) {
    delayMicroseconds(200); 
    //cycleRight() did not get reqMaster() to initiate a MRB2 to happen.
    //All other functions that was identical worked.
    //That's super weird, but now it works thanks to the delay above.
    reqMaster();
    reqMasterFlag = false;
  }

  if (runPeriodically == 0) {
    float battery = getBatV();
    String message = "BAT: " + String(battery, 1) + "V" + '\0';
    runPeriodically = 100000;
    textRow = 2;
    message.getBytes(customText[textRow - 1], 36);
    reqMaster();
  }
}



/*
                END LOOP
*/






//Notify HU that we want to trigger the first initiate procedure to add a new device
//(CD-CHGR/SAT etc) by pulling BUSY line low for 1s
void melbusInitReq() {
  //Serial.println("conn");
  //Disable interrupt on INT_NUM quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT_NUM);

  // Wait until Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(20);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  //timer0 is off so we have to do a trick here
  for (unsigned int i = 0; i < 12000; i++) delayMicroseconds(100);

  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  //Enable interrupt on INT_NUM, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT_NUM);
}


//This is a function that sends a byte to the HU - (not using interrupts)
//SET byteToSend variable before calling this!!
void SendByteToMelbus() {
  //Disable interrupt on INT_NUM quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT_NUM);

  //Convert datapin to output
  //pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
  DDRD |= (1 << MELBUS_DATA);

  //For each bit in the byte
  for (char i = 7; i >= 0; i--)
  {
    while (PIND & (1 << MELBUS_CLOCKBIT)) {} //wait for low clock
    //If bit [i] is "1" - make datapin high
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    }
    //if bit [i] is "0" - make datapin low
    else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    while (!(PIND & (1 << MELBUS_CLOCKBIT))) {}  //wait for high clock
  }
  //Let the value be read by the HU
  delayMicroseconds(20);
  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;
  DDRD &= ~(1 << MELBUS_DATA);

  //We have triggered the interrupt but we don't want to read any bits, so clear the flag
  EIFR |= 1 << INT_NUM;
  //Enable interrupt on INT_NUM, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT_NUM);
}


//This method generates our own clock. Used when in master mode.
void SendByteToMelbus2() {
  delayMicroseconds(700);
  //For each bit in the byte
  //char, since it will go negative. byte 0..255, char -128..127
  //int takes more clockcycles to update on a 8-bit CPU.
  for (char i = 7; i >= 0; i--)
  {
    delayMicroseconds(7);
    PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
    //If bit [i] is "1" - make datapin high
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    }
    //if bit [i] is "0" - make datapin low
    else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    //wait for output to settle
    delayMicroseconds(5);
    PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
    //wait for HU to read the bit
  }
  delayMicroseconds(20);
}



//Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
void MELBUS_CLOCK_INTERRUPT() {
  //Read status of Datapin and set status of current bit in recv_byte
  //if (digitalRead(MELBUS_DATA) == HIGH) {
  if ((PIND & (1 << MELBUS_DATA))) {
    melbus_ReceivedByte |= (1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "1"
  }
  else {
    melbus_ReceivedByte &= ~(1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "0"
  }

  //if all the bits in the byte are read:
  if (melbus_Bitposition == 0) {
    //set bool to true to evaluate the bytes in main loop
    byteIsRead = true;

    //Reset bitcount to first bit in byte
    melbus_Bitposition = 7;
  }
  else {
    //set bitnumber to address of next bit in byte
    melbus_Bitposition--;
  }
}


void SendText(byte rowNum) {
  //Disable interrupt on INT_NUM quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT_NUM);


  //Convert datapin and clockpin to output
  //pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
  PORTD |= 1 << MELBUS_DATA;      //set DATA input_pullup => HIGH when output (idle)
  DDRD |= (1 << MELBUS_DATA);     //set DATA as output
  PORTD |= 1 << MELBUS_CLOCKBIT;  //set CLK input_pullup => HIGH when output (idle)
  DDRD |= (1 << MELBUS_CLOCKBIT); //set CLK as output

  //send header
  for (byte b = 0; b < 4; b++) {
    byteToSend = textHeader[b];
    SendByteToMelbus2();
  }

  //send which row to show it on
  byteToSend = rowNum;
  SendByteToMelbus2();

  //send text
  for (byte b = 0; b < 36; b++) {
    byteToSend = customText[rowNum - 1][b];
    SendByteToMelbus2();
  }

  DDRD &= ~(1 << MELBUS_CLOCKBIT);//back to input (PULLUP since we clocked in the last bit with clk = high)
  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;  //this may or may not change the state, depending on the last transmitted bit
  DDRD &= ~(1 << MELBUS_DATA);//release the data line

  //Clear INT flag
  EIFR |= 1 << INT_NUM;
  //Enable interrupt on INT_NUM, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT_NUM);

  for (byte b = 0; b < 36; b++) {
    Serial.print(char(customText[rowNum - 1][b]));
  }
  Serial.println("  END");
}


void reqMaster() {
  DDRD |= (1 << MELBUS_DATA); //output
  PORTD &= ~(1 << MELBUS_DATA);//low
  delayMicroseconds(700);
  delayMicroseconds(800);
  delayMicroseconds(800);
  PORTD |= (1 << MELBUS_DATA);//high
  DDRD &= ~(1 << MELBUS_DATA); //back to input_pullup
}

void cycleRight() {
  rgtClr++;
  rgtClr &= 7;
  store();
  setLEDs();

  String ststxt = colors[rgtClr];
  for (byte b = 0; b < 36; b++) {
    customText[3][b] = 0;   //textRow 4 = index 3
  }
  ststxt.getBytes(customText[3], 7);
  textRow = 4;
  reqMasterFlag = true;
}

void cycleLeft() {
  lftClr++;
  lftClr &= 7;
  store();
  setLEDs();

  String ststxt = colors[lftClr];
  for (byte b = 0; b < 36; b++) {
    customText[2][b] = 0;   //textRow 3 = index 2
  }
  ststxt.getBytes(customText[2], 7);
  textRow = 3;
  reqMasterFlag = true;
}




void toggleOutput(byte pinNumber) {
  digitalWrite(pinNumber, !digitalRead(pinNumber));
  String ststxt = "";
  String txt[] = {
    "LFT ", "RGT ", "B", "G", "R"
  };

  byte pins = PINC & 0x3F;
  store();  //to EEPROM. Will toast the memory after 100 000 changes.
  for (byte b = 0; b < 5; b++) {
    if (pins & (1 << b)) {
      ststxt += txt[b];
    }
  }

  //reset customText (global variable to send to the HU)
  for (byte b = 0; b < 36; b++) {
    customText[0][b] = 0;
  }
  ststxt.getBytes(customText[0], 36);  //copy ststxt into customText
  textRow = 3;
  reqMasterFlag = true;
}

//Simulate button presses on the BT module. 200 ms works good. Less is not more in this case...
void nextTrack() {
  digitalWrite(nextPin, HIGH);
  for (byte i = 0; i < 200; i++)
    delayMicroseconds(1000);
  digitalWrite(nextPin, LOW);
}

void prevTrack() {
  digitalWrite(prevPin, HIGH);
  for (byte i = 0; i < 200; i++)
    delayMicroseconds(1000);
  digitalWrite(prevPin, LOW);
}

void play() {
  digitalWrite(playPin, HIGH);
  for (byte i = 0; i < 200; i++)
    delayMicroseconds(1000);
  digitalWrite(playPin, LOW);
}


//remember the state of the LEDs
void store() {
  EEPROM.update(EEPROM.length() - 2, lftClr);
  EEPROM.update(EEPROM.length() - 1, rgtClr);
}

//reset the LEDs to last saved state
void recall() {
  //PINC = (PINC & 0xC0) | EEPROM.read(EEPROM.length() - 1);
  lftClr = EEPROM.read(EEPROM.length() - 2);
  rgtClr = EEPROM.read(EEPROM.length() - 1);
  setLEDs();
}

void setLEDs() {
  //Faster and easier to use PORTC directly but this is more readable. Maybe ;-)
  //PORTC |= (lftClr)+(rgtClr<<3);
  digitalWrite(LED_RED_LFT, (lftClr & B00000100) >> 2);
  digitalWrite(LED_GRN_LFT, (lftClr & B00000010) >> 1);
  digitalWrite(LED_BLU_LFT, lftClr & B00000001);
  digitalWrite(LED_RED_LFT, (rgtClr & B00000100) >> 2);
  digitalWrite(LED_GRN_LFT, (rgtClr & B00000010) >> 1);
  digitalWrite(LED_BLU_LFT, rgtClr & B00000001);

}

void fixTrack() {
  //cut out A-F in each nibble, and skip "00"
  byte hn = track >> 4;
  byte ln = track & 0xF;
  if (ln == 0xA) {
    ln = 0;
    hn += 1;
  }
  if (ln == 0xF) {
    ln = 9;
  }
  if (hn == 0xA) {
    hn = 0;
    ln = 1;
  }
  if ((hn == 0) && (ln == 0)) {
    ln = 0x9;
    hn = 0x9;
  }
  track = (hn << 4) + ln;
}

void changeCD() {
  while (!(PIND & (1 << MELBUS_BUSY))) {
    if (byteIsRead) {
      byteIsRead = false;
      switch (melbus_ReceivedByte) {
        //0x81 to 0x86 corresponds to cd buttons 1 to 6 on the HU (650)
        case 0x81:
          cd = 1;
          track = 1;
          break;
        case 0x82:
          cd = 2;
          track = 1;
          break;
        case 0x83:
          cd = 3;
          track = 1;
          cycleLeft();
          break;
        case 0x84:
          cd = 4;
          track = 1;
          cycleRight();
          break;
        case 0x85:
          cd = 5;
          track = 1;
          break;
        case 0x86:
          cd = 6;
          track = 1;
          break;
        case 0x41:  //next cd
          cd++;
          track = 1;
          break;
        case 0x01:  //prev cd
          cd--;
          track = 1;
          break;
        default:
          track = 1;
          break;
      }
    }
  }
  trackInfo[3] = cd;
  trackInfo[5] = track;
}

void SendTrackInfo() {
  for (byte i = 0; i < 9; i++) {
    byteToSend = trackInfo[i];
    SendByteToMelbus();
  }
}

void SendCartridgeInfo() {
  for (byte i = 0; i < 6; i++) {
    byteToSend = cartridgeInfo[i];
    SendByteToMelbus();
  }
}

float getBatV() {
  float a, bv;

  a = analogRead(BATTERY) * 5.0 / 1023.0;
  bv = a * (R_1 + R_2 + R_CAR) / R_1 + T_LOSS;
  return bv;
}


//Happy listening AND READING, hacker!

