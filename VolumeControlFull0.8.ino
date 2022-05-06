#include <SPI.h>
#include <mcp_can.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>

#define ST77XX_GRAY 0x38e7 //00111 000111 00111
#define ST77XX_LGRAY 0x79EF //01111 011111 01111
/*
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800    11111 000000 00000
#define ST77XX_GREEN 0x07E0  00000 111111 00000
#define ST77XX_BLUE 0x001F   00000 000000 11111
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0 11111 111111 00000
#define ST77XX_ORANGE 0xFC00
*/
//new enc 
volatile unsigned long threshold = 3000;
//volatile int rotaryHalfSteps = 0;
volatile byte rotaryHalfSteps = 0x80;
//volatile long rotaryHalfStepsOld = 0;
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;

void int0() {
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, 2);
  if ( int0history == int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    rotaryHalfSteps++;
  else
    rotaryHalfSteps--;
}

void int1() {
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND, 3);
  if ( int1history == int1signal )
    return;
  int1time = micros();
}


char strLvl[8];
volatile byte lvlVol = 0x73; //start volume level
volatile byte lvlBass = 0xC9; //start volume level
byte lvlVolOld = 0x0;
byte lvlBassOld = 0x0;
volatile int lvlTemp = 0x00;
byte lvlVolCan=0x0e;

byte i = 0;
byte blnFrontCam = 1;
unsigned long timeFrontCamPressed = 0;

byte blnCamDisplayOn=0;
bool blnBass = true; //set to 1 when we need to change Bass level
bool blnBassOld = false;

bool blnDigitalInput = true;
bool blnDigitalInputOld = false;
byte blnEncButtonState=1;
byte blnEncButtonStatePrev=1;
bool blnVolByEnc = false;
unsigned long timeCurrent = 0;
unsigned long timeButtonPressed = 0;
unsigned long timeButtonPressedLastTime = 0;
unsigned long timeButtonPressedDuration = 0;


//unsigned long timeVolEnabled = 0;
byte buttonPressedCount=0;
byte intGear=0;
byte intGearPrev=0;
bool blnCANinitFailed=false;

char strInSerial[16];
char inChar = -1;

#define lvlStep 0x04
#define addrVol 0x11
#define addrBass 0x12
#define pinDigPot 9
#define pinCANShield 10
#define intCANShieldInitRetry 5
#define pinDigital 8
#define pinCamButton 7
#define pinCamRelay 6
#define pinEncButton 4
#define timeButtonWaitForInput 250
#define timeButtonDebounce 30
#define timeShortButtonPress 250
#define maxLevel 255
#define minLevel 0

//PINS
//Encoder pins
#define pinEncA 2
#define pinEncB 3


//TFT settings
#define TFT_DC    A1     // TFT DC  
#define TFT_RST   -1     // TFT RST 
#define TFT_CS    A0     // TFT CS  
#define tftColorBack ST77XX_BLACK
#define posTFTSerial 120
#define lblVol "Vol:"
#define posVolLblX 0
#define posVolLblY 5
#define posVolLvlX 50
#define posVolLvlY 5
#define posVolBarX 0
#define posVolBarY 24
#define posVolBarH 14
#define widthVolBar 240.0

#define lblBass "Bass:"
#define posBassLblX 0
#define posBassLblY 43
#define posBassLvlX 62
#define posBassLvlY 43
#define posBassBarX 0
#define posBassBarY 62

#define posDigInLblX 0
#define posDigInLblY 82
#define posDigInLvlX 170
#define posDigInLvlY 82

uint16_t colorVolBar = ST77XX_YELLOW;
uint16_t colorBassBar = ST77XX_YELLOW;
float fltBarWidthToVolLvl = widthVolBar/256.0;
byte intVolBarWidth = 0;


INT32U canId = 0x0;
unsigned char len = 0;
unsigned char buf[8];
MCP_CAN CAN(pinCANShield);

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* standard canId, in case we need only 1(or 2 canIDs)
    CAN.init_Mask(0, 0, 0x07ff); //bin ‭111 1111 1111‬    //1fffffff   
    CAN.init_Filt(0, 0, 0x740);  //canid1
    CAN.init_Filt(1, 0, 0x4c0);  //canid2

    CAN.init_Mask(1, 1, 0x1fffffff); //bin ‭111 1111 1111‬    //   
    CAN.init_Filt(2, 1, 0x17330b10);  //extcanid1
    CAN.init_Filt(3, 1, 0x17330a10);  //extcanid2
    CAN.init_Filt(4, 1, 0x17330a10);  //extcanid3
    CAN.init_Filt(5, 1, 0x17330a10);  //extcanid4

    CAN.init_Mask(0, 0, 0x0700); //bin ‭ 111 0000 0000‬
    CAN.init_Filt(0, 0, 0x700);  //all canids 0x7**
*/

void setup() {    
  Serial.begin(115200);
  pinMode(pinDigPot, OUTPUT);
  pinMode(pinEncButton, INPUT_PULLUP);
  pinMode(pinDigital, OUTPUT);
  
  pinMode(pinEncA, INPUT_PULLUP); 
  pinMode(pinEncB, INPUT_PULLUP); 
  pinMode(pinCamButton, INPUT_PULLUP);
  pinMode(pinCamRelay, OUTPUT);

  digitalWrite(pinDigital, blnDigitalInput);
  digitalWrite(pinCamRelay, blnFrontCam);
  digitalWrite(pinEncA, HIGH);
  digitalWrite(pinEncB, HIGH);
  attachInterrupt(0,int0,CHANGE);
  attachInterrupt(1,int1,CHANGE);

  SPI.begin();
  digitalPotWrite(lvlVol,addrVol); //set wiper to default
  digitalPotWrite(lvlBass,addrBass); //set wiper to default
  delay(100);
  tft.init(240, 240);    //tft.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
  tft.setRotation(2);// if the screen is flipped, remove this command
  tft.fillScreen(tftColorBack);
  tft.setTextColor(ST77XX_WHITE,tftColorBack);
  tft.setTextSize(2);
  tft.setCursor(posVolLblX, posVolLblY);
  tft.print(lblVol);
  tft.setCursor(posBassLblX, posBassLblY);
  tft.print(lblBass);
  tft.setTextColor(ST77XX_WHITE,tftColorBack);
  tft.setCursor(posDigInLblX, posDigInLblY);
  tft.print("Digital Input:");
  tft.setTextColor(ST77XX_GREEN,tftColorBack);

  tftSerialPrint("boot");
  START_CANBUS_INIT:
  if(CAN_OK == CAN.begin(CAN_500KBPS,MCP_8MHz)) {
    tftSerialPrint("CAN OK");
    delay(100);
    CAN.init_Mask(1, 1, 0x1fffffff);
    CAN.init_Filt(2, 1, 0x17333110);
    CAN.init_Filt(3, 1, 0x17330b00);
    CAN.init_Mask(0, 0, 0x07ff);
    CAN.init_Filt(0, 0, 0x05bf);
    CAN.init_Filt(1, 0, 0x3dc);
  } else {
    tftSerialPrint("CAN Failed");
    delay(300);
    if (i <= intCANShieldInitRetry) {
      i++;
      goto START_CANBUS_INIT;
    } else {
      blnCANinitFailed = true;
    }
  }
}

int digitalPotWrite(int value, int addr) {
  digitalWrite(pinDigPot, LOW);
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(pinDigPot, HIGH);
}

void tftSerialPrint(const char *strOut) {
  tftSerialPrint(strOut, true);
}

void tftSerialPrint(const char *strOut, bool blnLN) {
  if ((tft.getCursorY() > 230)||(tft.getCursorY() < posTFTSerial)) {
    tft.setCursor(0, posTFTSerial);
    tft.fillRect(0, posTFTSerial, 240, 240, tftColorBack);    
  }
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN,tftColorBack);
  if (blnLN) {
    tft.println(strOut);  
  } else {
    tft.print(strOut);
  }
}

void tftVolPrint(byte Level, byte pos){
  byte oldX = tft.getCursorX();
  byte oldY = tft.getCursorY();
  tft.setTextSize(2); //each symbol =10px + 2px space
  tft.setTextColor(ST77XX_GREEN,tftColorBack);

  switch (pos) {
  case 0: //Volume
    tft.setCursor(posVolLvlX, posVolLvlY);
    break;
  case 1: //Bass Level
    tft.setCursor(posBassLvlX, posBassLvlY);
    break;
  case 2: //Digital input
    tft.setCursor(posDigInLvlX, posDigInLvlY);
    if (Level == 0) {
      tft.print("On ");
    } else {
      tft.print("Off");
    }
    break;
  }
  if (pos < 2) {
    if (Level < 10) {
      tft.print("  ");
    } else if (Level < 100) {
      tft.print(" ");
    } 
    tft.print(Level);
  }
  tft.setCursor(oldX, oldY);
}

void tftVolBarPrint(byte lvlVolOld, byte lvlVol, byte lvlType, uint16_t intColor){
  tftVolPrint(lvlVol, lvlType);
  byte posX;
  byte posY;
  byte intVolBarWidth = fltBarWidthToVolLvl*lvlVol;
  if (lvlType == 0) {
    posX = posVolBarX;
    posY = posVolBarY;
  } else {
    posX = posBassBarX;
    posY = posBassBarY;
  }
  if (lvlVolOld > lvlVol) {
      tft.fillRect(intVolBarWidth, posY, tft.width()-intVolBarWidth, posVolBarH, tftColorBack);
  } else {
      tft.fillRect(posX, posY, intVolBarWidth, posVolBarH, intColor);
  } 
}



void loop(){  
  if (lvlVolOld != lvlVol) {
    digitalPotWrite(lvlVol,addrVol);
    tftVolBarPrint(lvlVolOld, lvlVol, 0, colorVolBar);
    lvlVolOld = lvlVol;
  }
  if (lvlBassOld != lvlBass) {
    digitalPotWrite(lvlBass,addrBass);
    tftVolBarPrint(lvlBassOld, lvlBass, 1, colorBassBar);
    lvlBassOld = lvlBass;
  }
  if (blnDigitalInput != blnDigitalInputOld) {
    tftVolPrint(blnDigitalInput,2);
    digitalWrite(pinDigital, blnDigitalInput);
    blnDigitalInputOld = blnDigitalInput;
  }
  if (blnBassOld != blnBass) {
    if (!blnBass) {
      colorVolBar = ST77XX_YELLOW;
      colorBassBar = ST77XX_LGRAY;
    } else {
      colorVolBar = ST77XX_LGRAY;
      colorBassBar = ST77XX_YELLOW;
    }
    tftVolBarPrint(lvlVolOld, lvlVol, 0, colorVolBar);
    tftVolBarPrint(lvlBassOld, lvlBass, 1, colorBassBar);
    blnBassOld = blnBass;
  }
  
  timeCurrent = millis();
  if (Serial.available() > 0) {
    inChar = Serial.read();
    if (inChar == 0x0a) {
      strInSerial[i] = '\0';
      tftSerialPrint("ESP: ", false);
      tftSerialPrint(strInSerial);
      if (strcmp(strInSerial, "<dig_enabled>") == 0) {
        blnDigitalInput = true;
      }      
      if (strcmp(strInSerial, "<dig_disabled>") == 0) {
        blnDigitalInput = false;
      }
      i = 0;
    } else {
      strInSerial[i] = inChar;
      if (inChar != 0x0d) {
        i++;
      }
    }
  }

  if ((timeCurrent-timeFrontCamPressed)>500) {
    if (digitalRead(pinCamButton) == LOW) {  
      tftSerialPrint("cam button pressed ");
      timeFrontCamPressed = timeCurrent;
      if (blnFrontCam) {
        blnFrontCam = LOW;
      } else {
        blnFrontCam = HIGH;
      }
      digitalWrite(pinCamRelay, blnFrontCam);
    }
  }
  blnEncButtonState = digitalRead(pinEncButton);

  if (blnEncButtonStatePrev != blnEncButtonState) {
    timeButtonPressedLastTime = timeCurrent;
    if (blnEncButtonStatePrev > blnEncButtonState) {
      timeButtonPressed = timeCurrent;
    } else {
      if ((timeCurrent-timeButtonPressed) > timeButtonDebounce) {
        buttonPressedCount++;
        timeButtonPressedDuration = timeCurrent - timeButtonPressed;
      }
    }
  }

  if ((buttonPressedCount > 0)&&((timeCurrent-timeButtonPressedLastTime) > timeButtonWaitForInput)) {
      if (buttonPressedCount == 1) {
        if (timeButtonPressedDuration < timeShortButtonPress) { //short enc button press
          blnBass = !blnBass;
        } else { //long enc button press
          if (!blnVolByEnc) {
            tftSerialPrint("Vol by encoder");
          } else {
            tftSerialPrint("Vol by CAN");
          }
          blnVolByEnc = !blnVolByEnc;
        }
      } else { //double click
        tftSerialPrint("DIG IN by Encoder");
        blnDigitalInput = !blnDigitalInput;
      }
      buttonPressedCount = 0;
  }
  blnEncButtonStatePrev = blnEncButtonState;


  if (!blnCANinitFailed) { 
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&len, buf);
      canId = CAN.getCanId();
        if (canId == 0x5bf) { //wheelbuttons
          if (buf[0] == 0x0c) {
              tftSerialPrint("DIG IN by CAN");
              blnDigitalInput = !blnDigitalInput;
          }
        } 
        if (canId == 0x3dc) {
          if ((buf[5] & 0x0F)==0x06) {
            intGear = 255;
          } else {
            if ((buf[5] & 0x0F)>0x07) {
              intGear = 1;
            }
          }
        }
        if ((canId == 0x17330b00)&&(buf[0] == 0x22)) {
          if (buf[2] == 0x11) {
            //tftSerialPrint("cam display enabled");
            blnCamDisplayOn=1;
          }
          if (buf[2] == 0x00) {
            //tftSerialPrint("cam display disabled");
            blnCamDisplayOn=0;
          }
        }
        if (canId == 0x17333110) {
          if (buf[1] == 0x6F) {
              if ((buf[0] == 0x4C)||(buf[0] == 0x3C)) {
                  lvlVolCan=buf[5]; //Vol
              }
          }
          if ((buf[1] == 0x52)&&(buf[0] == 0x3C)) {
                  lvlVolCan=buf[2]; //Vol  
          }
          if ((!blnDigitalInput)&&(!blnVolByEnc)) {
            lvlTemp = lvlVolCan*8.5;
            if (lvlTemp != lvlVol) {
               lvlVol = lvlTemp;
            }
          }
        }
    }
  }

  if (blnCamDisplayOn == 1) {
      if (intGear != intGearPrev) {
        intGearPrev = intGear;
        tftSerialPrint("gear changed from R to D or D to R");
          if ((intGear == 255)&&(blnFrontCam == LOW)){
            blnFrontCam = HIGH;
            digitalWrite(pinCamRelay, blnFrontCam);
            tftSerialPrint("disabling front cam");
          }
          if ((intGear == 1)&&(blnFrontCam == HIGH)){
            blnFrontCam = LOW;
            digitalWrite(pinCamRelay, blnFrontCam);
            tftSerialPrint("enabling front cam");
          }
      }
  } else {
          if (blnFrontCam == LOW){
            blnFrontCam = HIGH;
            digitalWrite(pinCamRelay, blnFrontCam);
            tftSerialPrint("disabling front cam because cam display disabling");
          }
  }
  
  if(rotaryHalfSteps != 0x80) {
    if (!blnBass) {
      lvlTemp = lvlVol;
    } else {
      lvlTemp = lvlBass;
    }
    
    lvlTemp = lvlTemp + (rotaryHalfSteps-0x80)*lvlStep;
    if (lvlTemp < minLevel) lvlTemp = minLevel;
    if (lvlTemp > maxLevel) lvlTemp = maxLevel;
    
    if (!blnBass) {
      lvlVol = lvlTemp;
    } else {
      lvlBass = lvlTemp;
    }
    rotaryHalfSteps = 0x80;
  }
}
