/* Tested on my HU-650
 *  
 */

#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)4     //Pin D4  - Data
#define MELBUS_BUSY (byte)5     //Pin D5  - Busy

//volatile variables used inside and outside of ISP
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;

byte byteToSend = 0;  //global to avoid unnecessary overhead
bool Connected = false;

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
#define ACK (byte)0x00;

//This list can't be too long. We only have so much time between the received bytes. (approx 500 us)
byte commands[][5] = {
  {ALT_ID, 0x1E, 0xEF},             //0, Cartridge info request. Respond with 6 bytes
  {ALT_ID, 0x1B, 0xE0, 0x01, 0x08}, //1, track info req. resp 9 bytes
  {BASE_ID, 0x1B, 0x2D, 0x40, 0x01}, //2, next track.
  {BASE_ID, 0x1B, 0x2D, 0x00, 0x01}, //3, prev track
  {BASE_ID, 0x1A, 0x50, 0x41},       //4, next cd, not verified (what buttons trigger this command?)
  {ALT_ID, 0x1A, 0x50, 0x01},       //5, prev cd, not verified
  {BASE_ID, 0x19, 0x2F},            //6, power up. resp ack (0x00), not verified
  {BASE_ID, 0x19, 0x22},            //7, power down. ack (0x00), not verified
  {BASE_ID, 0x19, 0x29},            //8, FFW. ack, not verified
  {BASE_ID, 0x19, 0x26},            //9, FRW. ack, not verified
  {BASE_ID, 0x19, 0x2E},            //10 scan mode. ack, not verified
  {BASE_ID, 0x19, 0x52},            //11 random mode. ack, not verified
  {BASE_ID, 0x19, 0x44},            //12 ??. ack (cd player only?)
  {BASE_ID, 0x19, 0x40},            //13 ??. ack (cd player only?)
  {BASE_ID, 0x1B, 0x2D, 0x40, 0x06},//14 ??. ack (cd player only?)
  {0x07, 0x1A, 0xEE},               //15 main init seq. wait for BASE_ID and respond with RESPONSE_ID.
  {0x00, 0x00, 0x1C, 0xED},         //16 secondary init req. wait for BASE_ID and respond with RESPONSE_ID.
  {0x00, 0x1C, 0xEC}                //17 master req broadcast. wait for MASTER_ID and respond with MASTER_ID. (not used in this sketch)
};

//keep track of the length of each command. (The two dimensional array above must have fixed width)
byte cmdLen[18] = {3, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 5, 3, 4, 3};

byte track = 0x01; //Display show HEX value, not DEC. (A-F not "allowed")
byte cd = 0x02; //1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)


void setup() {
  //Disable timer0 interrupt. It's is only bogging down the system. We need speed!
  TIMSK0 &= ~_BV(TOIE0);

  //All lines are idle HIGH
  pinMode(MELBUS_DATA, INPUT_PULLUP);
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  
  //LED indicates connected status.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //Activate interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);

  //Initiate serial communication to debug via serial-usb (arduino)
  //For debugging purpose. Better off without it when things work.
  //Serial printing take a lot of time!!
  //Serial.begin(115200);

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

  static byte byteCounter = 0;  //keep track of how many bytes is sent in current command
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static byte matching[18];     //Keep track of every matching byte in the commands array
  static byte tm1 = 0;          //timer0 is switched OFF, so we need other means of making long delays.
  static byte tm2 = 0;
  
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  //check BUSY line active (active low)
  while (!BUSY) {
    //Transmission handling here...
    if (byteIsRead) {
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < 18; cmd++) {   //Hardcoded array length here...
        //check if previous bytes was matching
        if (matching[cmd] == byteCounter) {
          //check if this byte is matching
          if (lastByte == commands[cmd][byteCounter]) {
            matching[cmd]++;
            //check if a complete command is received, and take appropriate action
            if (matching[cmd] == cmdLen[cmd]) {
              switch (cmd) {
                case 0:
                  SendCartridgeInfo();
                  break;
                case 1:
                  SendTrackInfo();
                  break;
                case 2:
                  track++;
                  fixTrack();
                  break;
                case 3:
                  track--;
                  fixTrack();
                  break;
                case 4:
                  cd++;
                  break;
                case 5:
                  cd--;
                  break;
                case 6:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 7:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 8:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 9:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 10:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 11:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 12:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 13:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 14:
                  byteToSend = 0x00;
                  SendByteToMelbus();
                  break;
                case 15:
                  //wait for base_id and respond with response_id
                  while (!(PIND & (1 << MELBUS_BUSY))) {
                    while (!byteIsRead) {}
                    byteIsRead = false;
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
                    }
                  }
                  break;
                case 16:
                  //wait for base_id and respond response_id
                  //digitalWrite(13, HIGH);
                  while (!(PIND & (1 << MELBUS_BUSY))) {
                    while (!byteIsRead) {}
                    byteIsRead = false;
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
                    }
                  }
                  break;
                case 17:
                  //wait for master_id and respond with master_id
                  //digitalWrite(13, HIGH);
                  while (!(PIND & (1 << MELBUS_BUSY))) {
                    while (!byteIsRead) {}
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      //do stuff here to send message to HU, like
                      //masterSend();
                      break;
                    }
                  }
                  break;
              }
              //Well, since HU is talking to us we might call it a conversation.
              Connected = true;
            }
          }
        }
      }
      byteCounter++;
    }
    //Update status of BUSY line, so we don't end up in an infinite while-loop.
    BUSY = PIND & (1 << MELBUS_BUSY);
  }

  //Do other stuff here if you want. Nothing is beeing transmitted now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.
  //Reset stuff
  byteCounter = 0;
  melbus_Bitposition = 7;
  for (byte i = 0; i < 18; i++) {
    matching[i] = 0;
  }
  
  //Count to 256*255 before whining about lost connection. This value has to be tuned...
  tm1++;
  if (tm1 = 0) {
    tm2++;
  }
  if (!Connected && (tm2 == 255)) {
    tm1 = 0;
    tm2 = 0;
    melbusInitReq();
  }
  if (Connected) {
    PORTB |= (1 << 5); //LED ON
  } else {
    PORTB &= ~(1 << 5); //LED OFF
  }
  Connected = false; //will be turned on again in the while loop above.
}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbusInitReq() {
  //Serial.println("conn");
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);

  // Wait untill Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(10);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);

  //timer0 is off so we have to do a trick here
  for (int i = 0; i < 12000; i++) delayMicroseconds(100);

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
  //01,22 = HU asking for status rapidly
  //11 = HU display: CD cartridge empty
  //44 = HU display: CD number is blinking
  //81 = HU display: cd error

  byteToSend = 0x01;
  SendByteToMelbus();
  //8 = HU display: random
  //could possibly be used to randomly switch songs :D

  byteToSend = cd;
  SendByteToMelbus();
  //HU display: CD number (1-10). 0x10 shows "CD10". 0x0A (DEC 10) does not work!
  //HU displays each nibble, and only values < A is valid.

  byteToSend = 0xE8;
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

void SendCartridgeInfo() {
  noInterrupts();
  //According to internet resources, HEX 00 08 01 4A 0C CC is a valid answer.
  byteToSend = 0x00;
  SendByteToMelbus();
  byteToSend = 0x08;
  SendByteToMelbus();
  byteToSend = 0x01;
  SendByteToMelbus();
  byteToSend = 0x03;
  SendByteToMelbus(); //cd number
  byteToSend = 0x04;
  SendByteToMelbus();
  byteToSend = 0xF0;
  SendByteToMelbus(); //trk minus5???
  interrupts();
}

void fixTrack() {
  //cut out A-F in each nibble, and skip "00"
  int hn = track >> 4;
  int ln = track & 0x0F;
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
  if (hn == 0 & ln == 0) {
    ln = 0x9;
    hn = 0x9;
  }
  track = (hn << 4) + ln;
}





