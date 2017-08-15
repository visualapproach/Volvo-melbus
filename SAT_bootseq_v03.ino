/*

  By Thomas Landahl, 2017-08-14
  iMIV sniffed by Vincent Gijsen. Thanks a LOT!

  changed back to my HW settings and made a new sendbytetomelbus function which generates its own clock
  line 200 has been updated (bug correction)
*/

#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)3     //Pin D3  - Data
#define MELBUS_BUSY (byte)4     //Pin D4  - Busy

const byte prevPin = 8;
const byte nextPin = 9;
const byte upPin = 10;    //volume up
const byte downPin = 11;  //volume down
const byte playPin = 12;

//volatile variables used inside and outside of ISP
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;


byte byteToSend = 0;  //global to avoid unnecessary overhead
unsigned long Connected = 0;


/*
  #define RESPONSE_ID 0xEE  //ID while responding to init requests (which will use base_id)
  #define MASTER_ID 0xEF    //ID while requesting/beeing master
  #define BASE_ID 0xE8      //ID when getting commands from HU
  #define ALT_ID 0xE9       //Alternative ID when getting commands from HU
*/
#define RESPONSE_ID 0xC6  //ID while responding to init requests (which will use base_id)
#define BASE_ID 0xC0      //ID when getting commands from HU
#define MASTER_ID 0xC7

byte textPayload[41] = {
  //  0xFC, 0xC6, 0x73, 0x01, 0x04, ' ', ' ',
  //  'b', 'i', 't', 'c', 'h', 'e', 's', 0x00, 0x00,
  //  0x20, 0x20, 0x20, 0x20, 0x20,
  //  0x20, 0x20, 0x20, 0x20, 0x20,
  //  0x20, 0x20, 0x20, 0x20, 0x20,
  //  0x20, 0x20, 0x20, 0x20, 0x20,
  //  0x20, 0x20, 0x20, 0x20, 0x20
  (0xFC), (0xC6), (0x73), (0x01), (0x03),
  (0x20), (0x20), (0x77), (0x77), (0x77),
  (0x2E), (0x69), (0x4D), (0x49), (0x56),
  (0x2E), (0x63), (0x61), (0x20), (0x20),
  (0x20), (0x00), (0x00), (0x20), (0x20),
  (0x20), (0x20), (0x20), (0x20), (0x20),
  (0x20), (0x20), (0x20), (0x20), (0x20),
  (0x20), (0x20), (0x20), (0x20), (0x20),
  (0x20)

};
const byte SO_TEXT = 41;

const byte C1_Init_1[] = {
  0x10, 0x10, 0xc3, 0x01,
  0x00, 0x81, 0x01, 0xff,
  0x00
};
const byte SO_C1_Init_1 = 9;

const byte C1_Init_2[] = {
  (0x10), (0x01), (0x81),
  (0x6E), (0x6F), (0x20),
  (0x69), (0x50), (0x6F),
  (0x64), (0x20)
};
const byte SO_C1_Init_2 = 11;


const byte C3_Init_0[] = {
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

const byte C3_Init_1[] = {
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

const byte C3_Init_2[] = {
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




//This list can't be too long. We only have so much time between the received bytes. (approx 700 us)
const byte commands[][6] = {
  {0x00, 0x1C, 0xEC},                     // 0 now we are master and can send stuff (like text) to the display!
  {0x07, 0x1A, 0xEE},                     // 1 main init
  {0x00, 0x00, 0x1C, 0xED},               // 2 sec init
  {0xC0, 0x1B, 0x76},                     // 3 follows: [0, 92, ff], OR [1,3 ,FF] OR [2, 5 FF]
//  {0xC0, 0x1C, 0x70, 0x00, 0x80, 0x01 },  // 4 (we respond 0x90) power on?
//  {0xC0, 0x1C, 0x70, 0x02, 0x80, 0x01 },  // 5 (we respond 0x10) power off?
  {0xC0, 0x1C, 0x70, 0x02},                 //4 wait 2 bytes and answer 0x90?
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     //5 bogus
  {0xC0, 0x1D, 0x76, 0x80, 0x00},         // 6 unknown. Answer: 0x10, 0x80, 0x92
  {0xC1, 0x1B, 0x7F, 0x01, 0x08},         // 7 respond with c1_init_1
  {0xC1, 0x1D, 0x73, 0x01, 0x81,  0x10},  // 8 last byte might be a CMD or answer from device
  {0xC3, 0x1F, 0x7C, 0x00},                // 9 respond with c3_init_0
  {0xC3, 0x1F, 0x7C, 0x01},                // 10 respond with c3_init_1
  {0xC3, 0x1F, 0x7C, 0x02},                // 11 respond with c3_init_2
//  {0xC0, 0x1C, 0x70, 0x02, 0x00, 0x00}      //12 unknown
};
//keep track of the length of each command. (The two dimensional array above have fixed width (padded with 0x00))
const byte listLen = 12;
byte cmdLen[listLen] = {
  3,    
  3,
  4,
  3,
  4,
  6,
  5,
  5,  
  6,
  4,
  4,
  4
};



void setup() {
  //Disable timer0 interrupt. It's is only bogging down the system. We need speed!
  TIMSK0 &= ~_BV(TOIE0);

  //All lines are idle HIGH
  pinMode(MELBUS_DATA, INPUT_PULLUP);
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);

  //Initiate serial communication to debug via serial-usb (arduino)
  //Better off without it when things work.
  //Serial printing takes a lot of time!!
  Serial.begin(115200);
  Serial.print("calling HU");

  //Activate interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);

  //Call function that tells HU that we want to register a new device
  melbusInitReq();
}

//Main loop
void loop() {
  static byte byteCounter = 0;  //keep track of how many bytes is sent in current command
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static byte matching[listLen];     //Keep track of every matching byte in the commands array
  byte melbus_log[99];  //main init sequence is 61 bytes long...
  bool flag = false;
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  Connected++;
  //check BUSY line active (active low)
  while (!BUSY) {
    //Transmission handling here...
    if (byteIsRead) {
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      //Well, since HU is talking to us we might call it a conversation.
      Connected = 0;
      melbus_log[byteCounter] = lastByte;
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < listLen; cmd++) {
        //check if this byte is matching
        if (lastByte == commands[cmd][byteCounter]) {
          matching[cmd]++;
          //check if a complete command is received, and take appropriate action
          if ((matching[cmd] == cmdLen[cmd]) && (byteCounter + 1 == cmdLen[cmd])) {
            byte cnt = 0;
            switch (cmd) {
              //0, MASTER
              case 0:
                //wait for master_id and respond with same
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    //                    byteCounter++;
                    //                    lastByte = melbus_ReceivedByte;
                    //                    melbus_log[byteCounter] = lastByte;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      SendText();
                      break;
                    }
                  }
                }

              //1, MAIN INIT
              case 1:
                //wait for base_id and respond with response_id
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    //                    byteCounter++;
                    //                    lastByte = melbus_ReceivedByte;
                    //                    melbus_log[byteCounter] = lastByte;
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
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
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                      break;
                    }
                  }
                }
                break;

              //some command. answer 0x10
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
                Serial.println("ack c0 1b 76");
                break;


              //power on?
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
                Serial.println("power?");
                break;

              //power off?
              case 5:
                // {0xC0, 0x1C, 0x70, 0x02, 0x80, 0x01 } we respond 0x10;
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    break;
                  }
                }
                Serial.println("power on?");
                break;

              // unknown command. Known answer 0x10, 0x80, 0x92
              case 6:
                byteToSend = 0x10;
                SendByteToMelbus();
                byteToSend = 0x80;
                SendByteToMelbus();
                byteToSend = 0x92;
                SendByteToMelbus();
                Serial.println("case 6");
                break;

              case 7:
                //                for (byte i = 0; i < SO_C1_Init_1; i++) {
                //                  byteToSend = C1_Init_1[i];
                //                  //SendByteToMelbus();
                //                }
                byteToSend =  0x10;
                SendByteToMelbus();
                byteToSend =  0x10;
                SendByteToMelbus();
                byteToSend =  0xc3;
                SendByteToMelbus();
                byteToSend =  0x01;
                SendByteToMelbus();
                byteToSend =  0x00;
                SendByteToMelbus();
                byteToSend =  0x81;
                SendByteToMelbus();
                byteToSend =  0x01;
                SendByteToMelbus();
                byteToSend =  0xff;
                SendByteToMelbus();
                byteToSend =  0x00;
                SendByteToMelbus();

                Serial.print("\n****C1 init 1: ");
                break;

              case 8:
                for (byte i = 0; i < SO_C1_Init_2; i++) {
                  byteToSend = C1_Init_2[i];
                  SendByteToMelbus();
                }
                Serial.println("\nC1 init 2");
                break;

              case 9:
                for (byte i = 0; i < SO_C3_Init_0; i++) {
                  byteToSend = C3_Init_0[i];
                  SendByteToMelbus();
                  //                  byteCounter++;
                  //                  melbus_log[byteCounter] = byteToSend;
                }
                Serial.println("\nC3 init 0");
                break;

              case 10:
                for (byte i = 0; i < SO_C3_Init_1; i++) {
                  byteToSend = C3_Init_1[i];
                  SendByteToMelbus();
                }
                Serial.println("\nC3 init 1");
                break;

              case 11:
                for (byte i = 0; i < SO_C3_Init_2; i++) {
                  byteToSend = C3_Init_2[i];
                  SendByteToMelbus();
                }
                Serial.println("\nC3 init 2");
                break;

              case 12:
                //byteToSend = 0x10;
                //SendByteToMelbus();
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


  //Do other stuff here if you want. MELBUS lines are free now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.
  //Printing transmission log (incoming, before responses)
  if (flag) {
    for (byte b = 0; b < byteCounter; b++) {
      Serial.print(melbus_log[b], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    //Serial.print(".");
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

  if (Serial.available() > 0) {
    Serial.read();
    Serial.println("\nText will be sent!");
    //next run, we want to send text!
    reqMaster();
  }

  flag = false; //don't print during next loop. Wait for new message to arrive first.
}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbusInitReq() {
  //Serial.println("conn");
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
  delayMicroseconds(20);
  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;
  DDRD &= ~(1 << MELBUS_DATA);

  //Enable interrupt on INT0, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);
}


//This method generates our own clock. Used when in master mode.
void SendByteToMelbus2() {
  //For each bit in the byte
  for (int i = 7; i >= 0; i--)
  {
    PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
    delayMicroseconds(10);
    //If bit [i] is "1" - make datapin high
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    }
    //if bit [i] is "0" - make datapin low
    else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
    delayMicroseconds(10);
  }
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



void SendText() {
  Serial.println("sendtext()");
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);

  DDRD |= (1 << MELBUS_BUSY);
  PORTD &= ~(1 << MELBUS_BUSY);
  //Convert datapin to output
  //pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
  DDRD |= (1 << MELBUS_DATA);
  //we need to change clock to output
  //PORTD &= ~(1 << MELBUS_CLOCKBIT); //make output low when changing to output?
  PORTD |= 1 << MELBUS_CLOCKBIT;
  DDRD |= (1 << MELBUS_CLOCKBIT); //output

  for (byte i = 0; i < SO_TEXT; i++) {
    byteToSend = textPayload[i];
    delayMicroseconds(800);
    SendByteToMelbus2();
  }

  DDRD &= ~(1 << MELBUS_CLOCKBIT);//back to input (PULLUP)
  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;
  DDRD &= ~(1 << MELBUS_DATA);
  DDRD &= ~(1 << MELBUS_BUSY);
  PORTD |= 1 << MELBUS_BUSY;

  //Enable interrupt on INT0, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);

  Serial.println("finished");
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



//Happy listening, hacker!


