/* Tested on my HU-650
    7c: eliminated prev bytes check = faster loop.
    serial: scanned some messages and corrected a command (base_id instead of alt_id)
    cartridge info gives error!

*/

#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)3     //Pin D3  - Data
#define MELBUS_BUSY (byte)4     //Pin D4  - Busy
const byte prevPin = 8;
const byte nextPin = 9;
const byte playPin = 12;

//volatile variables used inside and outside of ISP
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;

byte byteToSend = 0;  //global to avoid unnecessary overhead
unsigned long Connected = 0;

//Base adresses.
//const byte MD_ADDR = 0x70;  //internal
//const byte CD_ADDR = 0x80;  //internal
//const byte TV_ADDR = 0xA8;  //might be A9 during main init sequence
//const byte DAB_ADDR = 0xB8;
//const byte SAT_ADDR = 0xC0;
//const byte MDC_ADDR = 0xD8;
//const byte CDC_ADDR = 0xE8;
//const byte RESPONSE = 0x06; //add this to base adress when responding to HU
//const byte MASTER = 0x07;   //add this to base adress when requesting/beeing master
//const byte DEVICE = CDC_ADDR; //change this if you want to fake another device

//change theese definitions if you wanna emulate another device.
//My HU-650 don't accept anything but a CD-C (so far).
#define RESPONSE_ID 0xEE  //ID while responding to init requests (which will use base_id)
#define MASTER_ID 0xEF    //ID while requesting/beeing master
#define BASE_ID 0xE8      //ID when getting commands from HU
#define ALT_ID 0xE9       //Alternative ID when getting commands from HU
//#define HIGH_NIBBLE (byte)0xE;

//This list can't be too long. We only have so much time between the received bytes. (approx 500 us)
byte commands[][5] = {
  {BASE_ID, 0x1E, 0xEF},             //0, Cartridge info request. Respond with 6 bytes (confirmed)
  {ALT_ID, 0x1B, 0xE0, 0x01, 0x08}, //1, track info req. resp 9 bytes
  {BASE_ID, 0x1B, 0x2D, 0x40, 0x01}, //2, next track.
  {BASE_ID, 0x1B, 0x2D, 0x00, 0x01}, //3, prev track
  {BASE_ID, 0x1A, 0x50, 0x41},       //4, next cd, not verified (what buttons on HU trigger this command?)
  {ALT_ID, 0x1A, 0x50, 0x01},       //5, prev cd, not verified
  {BASE_ID, 0x19, 0x2F},            //6, power up. resp ack (0x00), response not verified. HU keeps sending this cmd.
  {BASE_ID, 0x19, 0x22},            //7, power down. ack (0x00), not verified
  {BASE_ID, 0x19, 0x29},            //8, FFW. ack, not verified
  {BASE_ID, 0x19, 0x26},            //9, FRW. ack, not verified
  {BASE_ID, 0x19, 0x2E},            //10 scan mode. ack, cmd verified!
  {BASE_ID, 0x19, 0x52},            //11 random mode. ack, not verified
  {0x07, 0x1A, 0xEE},               //12 main init seq. wait for BASE_ID and respond with RESPONSE_ID.
  {0x00, 0x00, 0x1C, 0xED},         //13 secondary init req. wait for BASE_ID and respond with RESPONSE_ID.
  {0x00, 0x1C, 0xEC}                //14 master req broadcast. wait for MASTER_ID and respond with MASTER_ID.
  //  {BASE_ID, 0x19, 0x44},            //15 ??. ack (cd player only?)
  //  {BASE_ID, 0x19, 0x40},            //16 ??. ack (cd player only?)
  //  {BASE_ID, 0x1B, 0x2D, 0x40, 0x06},//17 ??. ack (cd player only?)
};


//keep track of the length of each command. (The two dimensional array above have fixed width (padded with 0x00))
const byte listLen = 15;
byte cmdLen[listLen] = {3, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 3, 3, 4, 3};

//predefined values
byte track = 0x01; //Display show HEX value, not DEC. (A-F not "allowed")
byte cd = 0x01; //1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)
byte powerup_ack = 0x00;


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
  digitalWrite(prevPin, LOW);
  digitalWrite(playPin, LOW);
  

  //LED indicates connected status.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //Activate interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);

  //Initiate serial communication to debug via serial-usb (arduino)
  //For debugging purpose. Better off without it when things work.
  //Serial printing take a lot of time!!
  Serial.begin(115200);

  //Call function that tells HU that we want to register a new device
  melbusInitReq();
}

//Main loop
void loop() {
  /* Note to self
     check for busy going low, then data will come. Fast.
     When high again, restart all counters.
     keep count of bytecount in current broadcast.
  */

  //it MIGHT be faster to keep theese variables global.
  static byte byteCounter = 0;  //keep track of how many bytes is sent in current command
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static byte matching[listLen];     //Keep track of every matching byte in the commands array
  //static byte msgCount = 0;          //inc every time busy line goes idle.
  //static byte masterCount = 0;
  byte melbus_log[99];  //main init sequence is 61 bytes long...
  bool flag = false;
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  Connected++;
  //check BUSY line active (active low)
  while (!BUSY) {
    //Transmission handling here...
    //debug
    //PORTB |= 1 << 5;
    if (byteIsRead) {
      //debug
      //PORTB &= ~(1 << 5);
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      //Well, since HU is talking to us we might call it a conversation.
      Connected = 0;
      melbus_log[byteCounter] = lastByte;
      //Serial.print(lastByte, HEX);
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < listLen; cmd++) {
        //check if this byte is matching
        if (lastByte == commands[cmd][byteCounter]) {
          matching[cmd]++;
          //check if a complete command is received, and take appropriate action
          if (matching[cmd] == cmdLen[cmd]) {
            switch (cmd) {
              //0, Cartridge info request. Respond with 6 bytes
              case 0:
                SendCartridgeInfo();
                break;
              //1, track info req. resp 9 bytes
              case 1:
                SendTrackInfo();
                //Serial.println("  trk info sent");
                break;
              //2, next track.
              case 2:
                track++;
                fixTrack();
                nextTrack();
                break;
              //3, prev track
              case 3:
                track--;
                fixTrack();
                prevTrack();
                break;
              //4, next cd, not verified (what buttons trigger this command?)
              case 4:
                cd++;
                break;
              //5, prev cd, not verified
              case 5:
                cd--;
                break;
              //6, power up. resp ack (0x00), not verified
              case 6:
                byteToSend = 0x00;
                SendByteToMelbus();
                //SendPowerUp();
                //powerup_ack++; //test until HU is satisfied
                //Serial.println(powerup_ack);
                break;
              //7, power down. ack (0x00), not verified
              case 7:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //8, FFW. ack, not verified
              case 8:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //9, FRW. ack, not verified
              case 9:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //10 scan mode. ack, not verified
              case 10:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //11 random mode. ack, not verified
              case 11:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //12 main init seq. wait for BASE_ID and respond with RESPONSE_ID.
              case 12:
                //wait for base_id and respond with response_id
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    //debug get whole message
                    //byteCounter++;
                    //melbus_log[byteCounter] = melbus_ReceivedByte;
                    //end debug
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
                    }
                  }
                }
                break;
              //13 secondary init req. wait for BASE_ID and respond with RESPONSE_ID.
              case 13:
                //wait for base_id and respond response_id
                //digitalWrite(13, HIGH);
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    //debug get whole message
                    //byteCounter++;
                    //melbus_log[byteCounter] = melbus_ReceivedByte;
                    //end debug
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
                    }
                  }
                }
                break;
              //14 master req broadcast. wait for MASTER_ID and respond with MASTER_ID. (not used in this sketch)
              case 14:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      //do stuff here to send message to HU, like
                      masterSend();
                      break;
                    }
                  }
                }
                break;
              //15 ??. ack (cd player only?)
              case 15:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //16 ??. ack (cd player only?)
              case 16:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
              //17 ??. ack (cd player only?)
              case 17:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
            }
            break;    //bail for loop. (Not meaningful to search more commands if one is already found)
          } //end if command found
        } //end if lastbyte matches
      }  //end for cmd loop
      byteCounter++;
    }  //end if byteisread
    //Update status of BUSY line, so we don't end up in an infinite while-loop.
    BUSY = PIND & (1 << MELBUS_BUSY);
    if (BUSY) {
      flag = true; //used to execute some code only once between transmissions
    }
  }

  //Do other stuff here if you want. Nothing is beeing transmitted now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.
  //Printing transmission log (incoming, before responses)
  if (flag) {
    for (byte b = 0; b < byteCounter; b++) {
      Serial.print(melbus_log[b], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  //Reset stuff
  byteCounter = 0;
  melbus_Bitposition = 7;
  for (byte i = 0; i < listLen; i++) {
    matching[i] = 0;
  }
  if (Connected > 2000000) {
    melbusInitReq();
    Connected = 0;
  }

  // I haven't seen any advantages from sending messages to HU.
  //Therefore this section is disabled.
  //  if (flag) {
  //    if(some timed interval etc)
  //    reqMaster();
  //  }

  flag = false; //don't print during next loop. Wait for new message to arrive first.
}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbusInitReq() {
  Serial.println("conn");
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);

  // Wait until Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(20);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  //timer0 is off so we have to do a trick here
  for (unsigned int i = 0; i < 12000; i++) delayMicroseconds(100);

  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  //Enable interrupt on INT0, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);
}


//This is a function that sends a byte to the HU - (not using interrupts)
//SET byteToSend variable before calling this!!
void SendByteToMelbus() {
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);

  //Convert datapin to output
  //pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
  DDRD |= (1 << MELBUS_DATA);

  //For each bit in the byte
  for (int i = 7; i >= 0; i--)
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

  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;
  DDRD &= ~(1 << MELBUS_DATA);

  //Enable interrupt on INT0, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);
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


void SendTrackInfo() {
  noInterrupts();
  //According to internet resources, HEX 10 88 01 01 80 03 00 02 22 should be a valid answer.
  //another source: 0x00, 0x02, 0x00, 0x01, 0x80, 0x01, 0xff, 0x60, 0x60
  //All comments in HEX

  //It would probably be faster to call a new function SendByteArrayToMelbus() instead of
  //calling SendByteToMelbus several times, with overhead each time. It seems we don't have to though,
  //because it works.

  //Sending 9 bytes:
  byteToSend = 0x94;
  SendByteToMelbus();
  //10, 14, 20, 24, 30, 34, 80, 84, 90, 94 => HU is OK, else error

  byteToSend = 0x01;
  SendByteToMelbus();
  //Is this a status message?
  //above line results:
  //01 = able to select cd by pushing HU-buttons but it resets to 01 after a couple of secs
  //00, 10, 20, 40 = HU display: cd load cartridge
  //22 = HU asking for status rapidly
  //11 = HU display: CD cartridge empty
  //44 = HU display: CD number is blinking
  //81 = HU display: cd error

  byteToSend = 0x01;
  SendByteToMelbus();
  //8 = HU display: random
  //could possibly be used to randomly switch songs :D

  byteToSend = cd;
  SendByteToMelbus();
  //HU display: CD number (1-10). 0x10 shows "CD  ". 0x0A (DEC 10) does not work!
  //HU displays each nibble, and only values < A is valid.

  byteToSend = 0x80;
  SendByteToMelbus();
  //Unknown meaning. Could this be unit ID? Like E8
  // < 40 (HEX) shows TRK-nn (minus sign)

  byteToSend = track;
  SendByteToMelbus();
  //TRACK #

  byteToSend = 0x00;
  SendByteToMelbus(); //probably hours
  byteToSend = 0x03;
  SendByteToMelbus(); //probably minutes
  byteToSend = 0x01;
  SendByteToMelbus(); //probably seconds  never shown on my HU though ;-(
  interrupts();
}

void reqMaster() {
  DDRD |= (1 << MELBUS_DATA); //output
  PORTD &= ~(1 << MELBUS_DATA);//low
  delayMicroseconds(700);
  delayMicroseconds(700);
  delayMicroseconds(800);
  PORTD |= (1 << MELBUS_DATA);//high
  DDRD &= ~(1 << MELBUS_DATA); //back to input
}

void SendCartridgeInfo() {
  noInterrupts();
  //According to internet resources, HEX 00 08 01 4A 0C CC is a valid answer.
  //another source: 0x00, 0x0f, 0xff, 0x4a, 0xfc, 0xff
  byteToSend = 0x00;
  SendByteToMelbus();
  byteToSend = 0x0F;
  SendByteToMelbus();
  byteToSend = 0xFF;
  SendByteToMelbus();
  byteToSend = 0x4A;
  SendByteToMelbus(); //cd number
  byteToSend = 0xFC;
  SendByteToMelbus();
  byteToSend = 0xFF;
  SendByteToMelbus();
  interrupts();
}

void SendPowerUp() {
  //This is just a test. Not in use. When I sniffed the data between
  //HU and CD-player, the CD-player answered like this, but it might have
  //been a glitch in the comm.
  noInterrupts();
  byteToSend = 0x00;
  SendByteToMelbus();
  //  byteToSend = 0x8D;
  //  SendByteToMelbus();
  //  byteToSend = 0xF0;
  //  SendByteToMelbus();
  //  byteToSend = 0x00;
  //  SendByteToMelbus();
  //  byteToSend = 0x84;
  //  SendByteToMelbus();
  //  byteToSend = 0x18;
  //  SendByteToMelbus();
  //  byteToSend = 0x7F;
  //  SendByteToMelbus();
  //  byteToSend = 0xFF;
  //  SendByteToMelbus();
  //  //byteToSend = 0xFF;
  //  SendByteToMelbus();
  //  //byteToSend = 0xFF;
  //  SendByteToMelbus();
  //  //byteToSend = 0xFF;
  //  SendByteToMelbus();
  //  //byteToSend = 0xFF;
  //  SendByteToMelbus();
  //  SendByteToMelbus();
  //  SendByteToMelbus();
  interrupts();
}

void masterSend() {
  noInterrupts();
  byteToSend = 0xF8;
  SendByteToMelbus();
  byteToSend = 0x85;
  SendByteToMelbus();
  byteToSend = 0xE2;
  SendByteToMelbus();
  byteToSend = 0x80;
  SendByteToMelbus(); //cd number?
  byteToSend = 0x03;
  SendByteToMelbus();
  byteToSend = 0x00;
  SendByteToMelbus();
  byteToSend = 0x02;
  SendByteToMelbus();
  byteToSend = 0x02;
  SendByteToMelbus();
  interrupts();
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

//Simulate button presses on the BT module. 200 ms works good. Less is not more in this case...
void nextTrack() {
  digitalWrite(nextPin, HIGH);
  for(byte i = 0; i < 200; i++)
  delayMicroseconds(1000);
  digitalWrite(nextPin, LOW);
}

void prevTrack() {
  digitalWrite(prevPin, HIGH);
  for(byte i = 0; i < 200; i++)
  delayMicroseconds(1000);
  digitalWrite(prevPin, LOW);
}

void play() {
  digitalWrite(playPin, HIGH);
  for(byte i = 0; i < 200;i++)
  delayMicroseconds(1000);
  digitalWrite(playPin, LOW);
}

//Happy listening, hacker!


