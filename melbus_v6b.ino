
/* Melbus CDCHGR Emulator
   Program that emulates the MELBUS communication from a CD-changer (CD-CHGR) in a Volvo V70 (HU-xxxx) to enable AUX-input through the 8-pin DIN-contact.
   Tis setup is using an Arduino Nano 5v clone

   The HU enables the CD-CHGR in its source-menue after a successful initialization procedure is accomplished.
   The HU will remove the CD-CHGR everytime the car starts if it wont get an response from CD-CHGR (second init-procedure).

   Karl Hagström 2015-11-04

   This project went really smooth thanks to these sources:
   http://volvo.wot.lv/wiki/doku.php?id=melbus
   https://github.com/festlv/screen-control/blob/master/Screen_control/melbus.cpp
   http://forums.swedespeed.com/showthread.php?50450-VW-Phatbox-to-Volvo-Transplant-(How-To)&highlight=phatbox
*/

/*
 * Based upon Karl's work http://gizmosnack.blogspot.se/2015/11/aux-in-volvo-hu-xxxx-radio.html
 * and the links above. I figured out how to send cd and track number to HU and also made the code faster (and less human readable).
 * The arduino can also go into master mode (this is not necessary to send track information), but I don't know what to send.
 * I only managed to fake a CD-C. No text, nor MD-C.
 * You can change tracks on the HU and it will show in the display (it won't actually change the played track yet. I'm waiting for a BT device to arrive and I might
 * get it to play next/previous song)
 */

#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)4     //Pin D4  - Data
#define MELBUS_BUSY (byte)5     //Pin D5  - Busy

volatile uint8_t melbus_ReceivedByte = 0;
volatile uint8_t melbus_Bitposition = 7;
uint8_t ByteToSend = 0; //why is this global? Is it faster because cpu don't need to copy value? Dont have it as argument when global!!
volatile bool ByteIsRead = false;
bool Connected = false;
//bool masterBroadcast = false;
//volatile bool testbool = false;
//volatile bool AllowInterruptRead = true;

//Base adresses.
//const byte MD_ADDR = 0x70;  //internal
//const byte CD_ADDR = 0x80;  //internal
//const byte TV_ADDR = 0xA8;  //A9 during main init sequence, but we only check high nibble
//const byte DAB_ADDR = 0xB8;
//const byte SAT_ADDR = 0xC0;
//const byte MDC_ADDR = 0xD8;
//const byte CDC_ADDR = 0xE8;
//const byte RESPONSE = 0x06; //add this to base adress when responding to HU
//const byte MASTER = 0x07;   //add this to base adress when requesting/beeing master
//const byte DEVICE = CDC_ADDR; //change this if you want to fake another device

//This is quicker than reading a const. Every uS counts.
#define RESPONSE 0x06 //add this to base adress when responding to HU
#define MASTER 0x07   //add this to base adress when requesting/beeing master
#define DEVICE 0xE8 //change this if you want to fake another device
byte track = 0x01; //Display show HEX value, not DEC. (A-F not "allowed")
byte cd = 0x01; //1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)

static bool requestMaster = false;
bool flag = false;


//Startup sequence
void setup() {
  //Data is deafult input high
  pinMode(MELBUS_DATA, INPUT_PULLUP);

  //Activate interrupt on clock pin (INT0, D3)
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);
  //Set Clockpin-interrupt to input
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);

  //Initiate serial communication to debug via serial-usb (arduino)
  Serial.begin(57600);
  //Serial.println("Initiating contact with Volvo-Melbus:");

  //Call function that tells HU that we want to register a new device
  melbus_Init_Device();
}

//Main loop
void loop() {
  if (ByteIsRead) {
    //DEBUG
    //if (flag) Serial.println(melbus_ReceivedByte, HEX);
    //Reset bool to enable reading of next byte
    ByteIsRead = false;

    switch (melbus_ReceivedByte) {
      case 0x00:
        //"MASTER REQUEST BROADCAST" OR "SECONDARY INIT"
        while (!ByteIsRead); //wait for next byte
        ByteIsRead = false;
        //Serial.println("byte ");
        //Serial.println(melbus_ReceivedByte, HEX);

        switch (melbus_ReceivedByte) {

          //secondary init
          case 0x00:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            switch (melbus_ReceivedByte) {
              //secondary init
              case 0x1C:
                while (!ByteIsRead); //wait for next byte
                ByteIsRead = false;
                switch (melbus_ReceivedByte) {
                  //secondary init
                  case 0xED:
                    for (int i = 0; i < 10; i++) { //check next 100 bytes for our ID
                      while (!ByteIsRead); //wait for next byte
                      ByteIsRead = false;
                      if (melbus_ReceivedByte == DEVICE) { //is it my id?
                        SendByteToMelbus(DEVICE + RESPONSE);
                        break;
                      }
                    }
                    break;
                }
                break;
            }

          //master request broadcast (must sink DATA for 2 ms (while busy is low) to get HU to send this msg)
          case 0x1C:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            if (melbus_ReceivedByte == 0xEC) {
              for (int i = 0; i < 100; i++) { //check next 100 bytes for our master ID
                while (!ByteIsRead); //wait for next byte
                ByteIsRead = false;
                //Serial.println(melbus_ReceivedByte, HEX);
                if (melbus_ReceivedByte == DEVICE + MASTER) { //is it my master id?
                  SendByteToMelbus(DEVICE  + MASTER);
                  MasterSend();
                  Serial.println("master");
                  break;
                }
              }
            }
            break;

        }
        break;

      case 0x07:
        //"MAIN INITSEQUENCE"
        while (!ByteIsRead); //wait for next byte
        ByteIsRead = false;
        switch (melbus_ReceivedByte) {
          case 0x1A:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            switch (melbus_ReceivedByte) {
              case 0xEE:
                for (int i = 0; i < 100; i++) { //check next 100 bytes for our ID
                  while (!ByteIsRead); //wait for next byte
                  ByteIsRead = false;
                  if (melbus_ReceivedByte == DEVICE) { //is it my master id?
                    SendByteToMelbus(DEVICE  + RESPONSE);
                    break;
                  }
                }
                break;
            }
        }
        break;

      case DEVICE:
        //I don't think HU will call a base adress (except maybe TV)
        break;

      case DEVICE + 1:
        //COMMAND
        while (!ByteIsRead); //wait for next byte
        ByteIsRead = false;
        switch (melbus_ReceivedByte) {
          case 0x1E:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            //cartridge info
            switch (melbus_ReceivedByte) {
              case 0xEF:
                //send cartridge info to HU
                SendCartridgeInfo();
                break;
            }
            break;
          case 0x1B:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            //other commands
            switch (melbus_ReceivedByte) {
              case 0xE0:
                while (!ByteIsRead); //wait for next byte
                ByteIsRead = false;
                switch (melbus_ReceivedByte) {
                  case 0x01:
                    while (!ByteIsRead); //wait for next byte
                    ByteIsRead = false;
                    switch (melbus_ReceivedByte) {
                      case 0x08:
                        SendTrackInfo();
                        Connected = true;
                        break;
                    }
                    break;
                }
            }
            break;

//          default:
//            //DEBUG/EXPLORE other (unknown) commands. Show next 50 bytes of data.
//            //Comment these lines out when you know enough.
//            Serial.println(melbus_ReceivedByte, HEX);
//            for (int i = 0; i < 50; i++) { //check next 50 bytes for our ID
//              while (!ByteIsRead); //wait for next byte
//              ByteIsRead = false;
//              Serial.println(melbus_ReceivedByte, HEX);
//            }
//            break;
        }
        break;
      case 0x1B:
        while (!ByteIsRead); //wait for next byte
        ByteIsRead = false;
        //other commands
        switch (melbus_ReceivedByte) {
          case 0x2D:
            while (!ByteIsRead); //wait for next byte
            ByteIsRead = false;
            switch (melbus_ReceivedByte) {
              case 0x40:
                while (!ByteIsRead); //wait for next byte
                ByteIsRead = false;
                switch (melbus_ReceivedByte) {
                  case 0x01:
                    //next track
                    track++;
                    fixTrack();
                    break;
                }
                break;
              case 0x00:
                while (!ByteIsRead); //wait for next byte
                ByteIsRead = false;
                switch (melbus_ReceivedByte) {
                  case 0x01:
                    //previous track
                    track--;
                    fixTrack();
                    break;
                }
                break;
            }
            break;
        }

//      default:
//        Serial.print("unknown: ");
//        Serial.println(melbus_ReceivedByte, HEX);

    } //end of first switch
  } //end of if(byteisread)


  //If BUSYPIN is HIGH => HU is in between transmissions
  if (digitalRead(MELBUS_BUSY) == HIGH)  {
    //Make sure we are in sync when reading the bits by resetting the clock reader
    melbus_Bitposition = 7;

    if (millis() % 5000 == 0) {
      if (!Connected) {
        melbus_Init_Device();
      } else {
        //Serial.println("entering master mode");
        requestMaster = true;
      }
    }
  }

  //Pull data line low 2 ms to trigger master request. This doesn't work
  //in the above IF-statement, so I guess busy line needs to be low/busy strange enough.
  if (requestMaster) {
    //we can't request every time busy-line is high, so set requestMaster=true at some interval (5 secs)
    //DDRD |= (1 << MELBUS_BUSY); //output
    //PORTD &= ~(1 << MELBUS_BUSY);//low
    Serial.println();
    Serial.println("pulling data low");
    DDRD |= (1 << MELBUS_DATA); //output
    PORTD &= ~(1 << MELBUS_DATA);//low
    delayMicroseconds(2200);
    //PORTD |= (1 << MELBUS_BUSY);//high
    //DDRD &= ~(1 << MELBUS_BUSY); //back to input
    PORTD |= (1 << MELBUS_DATA);//high
    DDRD &= ~(1 << MELBUS_DATA); //back to input
    requestMaster = false;
    flag = true;
  }

}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbus_Init_Device() {
  Serial.println("initiating device");
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);

  // Wait untill Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(10);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  delay(1200);
  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);

  //Enable interrupt on INT0, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);
}

//This is a function that sends a byte to the HU - (not using interrupts)
void SendByteToMelbus(uint8_t byteToSend) {
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

  //Enable interrupt on INT0, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
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
    ByteIsRead = true;

    //Reset bitcount to first bit in byte
    melbus_Bitposition = 7;
  }
  else {
    //set bitnumber to address of next bit in byte
    melbus_Bitposition--;
  }
}

void MasterSend() {
  //we are master now. generate clock signal and transmit data. 7 or 8 bytes it seems. first 3 is command, then data?
  //HU driving busy line low.
//  Serial.println("sending master data");
//  byte msg[7] = {0x18, 0x84, 0xE1, 0x8D, 0x01, 0x01, 0x01};
  byte msg[8] = {0x18, 0x84, 0xE9, 0x8D, 0x01, 0x01, 0x01, 0x08};
  //Disable interrupt on INT0 quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT0);
  //DDRD |= (1 << MELBUS_BUSY); //output
  //PORTD &= ~(1 << MELBUS_BUSY);//low
  DDRD |= (1 << MELBUS_DATA); //output
  DDRD |= (1 << MELBUS_CLOCKBIT); //output

  for (int by = 0; by < 7; by++) {
    for (int bi = 7; bi >= 0; bi--) {
      PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
      if (msg[by] & (1 << bi)) {
        PORTD |= (1 << MELBUS_DATA); //set bit high
      } else {
        PORTD &= ~(1 << MELBUS_DATA);  //set bit low
      }
      delayMicroseconds(10);
      PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
      delayMicroseconds(20);
    }
    delayMicroseconds(150);
  }
  DDRD &= ~(1 << MELBUS_DATA); //back to input
  PORTD |= (1 << MELBUS_DATA); //INPUT_PULLUP
  DDRD &= ~(1 << MELBUS_CLOCKBIT);//back to input

  //How does HU know when we are finished???? timeout or do we need to fiddle with busy-line?
  //DDRD &= ~ (1 << MELBUS_BUSY); //input again
  //PORTD |= (1 << MELBUS_BUSY);  //release line = high
  //Enable interrupt on INT0, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT0);
}

void SendTrackInfo() {
  noInterrupts();
  //HEX 10 88 01 01 80 03 00 02 22 is a valid answer.
  SendByteToMelbus(0x94);
  //HEX 10, 14, 20, 24, 30, 34, 80, 84, 90, 94 is OK, else error
  SendByteToMelbus(0x01); //status?
  //above line results (HEX):
  //x01 = able to select cd but it resets to 01 after a couple of secs 
  //TODO: (sniff data to figure out when to change track accordingly)
  //00, 10, 20, 40=cd load cartridge
  //01,22 = HU asking for status rapidly
  //11 = CD cartridge empty
  //44 = CD number is blinking
  //81 = cd error
  SendByteToMelbus(0x01);
  //HEX 8 = random
  SendByteToMelbus(cd); //CD # in HEX, shown as DEC
  SendByteToMelbus(0xFF);
  // < 40 (HEX) shows TRK-# (minus sign)
  SendByteToMelbus(track); //TRACK #
  SendByteToMelbus(0x10); //probably hours
  SendByteToMelbus(0x03); //probably minutes
  SendByteToMelbus(0x01); //probably seconds  never shown though ;-(
  interrupts();
}

void SendCartridgeInfo() {
  //never got this to run (HU never asks)
  Serial.println("SENDING cartridge info");
  noInterrupts();
  //HEX 00 08 01 4A 0C CC is a valid answer.
  SendByteToMelbus(0x00);
  SendByteToMelbus(0x08);
  SendByteToMelbus(0x01);
  SendByteToMelbus(0x4A);
  SendByteToMelbus(0x0C);
  SendByteToMelbus(0xCC);
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
  //  if (hn == 0xF) {
  //    hn = 0x9;
  //    ln = 0x9;
  //  }
  if (hn == 0 & ln == 0) {
    ln = 0x9;
    hn = 0x9;
  }
  track = (hn << 4) + ln;
}





