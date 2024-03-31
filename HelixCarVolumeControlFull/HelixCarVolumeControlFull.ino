/*ver 0.924
JAN2024 updated vol sniffer. added code to write control status to MFD
bug fixes
canidMFD = 0x17333111
*/
#include <EEPROM.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>

#define CANEnabled 1 //0-No CAN module installed 1-CAN installed

#define canidMFD 0x17333111

#define ST77XX_GRAY 0x38e7 //00111 000111 00111
#define ST77XX_LGRAY 0x79EF //01111 011111 01111
#define ST77XX_BLACK 0x0000
/*
#define ST77XX_WHITE 0xFFFF #define ST77XX_RED 0xF800    11111 000000 00000 #define ST77XX_GREEN 0x07E0  00000 111111 00000 #define ST77XX_BLUE 0x001F   00000 000000 11111 #define ST77XX_CYAN 0x07FF #define ST77XX_MAGENTA 0xF81F #define ST77XX_YELLOW 0xFFE0 11111 111111 00000 #define ST77XX_ORANGE 0xFC00
*/

//GLOBAL PIN DEFINITION
//Encoder pins
#define pinEncA 2
#define pinEncB 3
#define pinEncButton 4

#define pinCANShield 10
#define pinDigPot 9    //Digital Pot Pins
#define pinDigital 8   //Digital out switch
#define pinCamButton 7 //cam change input button
#define pinCamRelay 6  //cam change relay

//GLOBAL PIN DEFINITION

char strLvl[8];
volatile byte lvlVol = 0x73; //start volume level
volatile byte lvlSub = 0xC9; //start volume level
byte lvlVolOld = 0x0;
byte lvlSubOld = 0x0;
volatile int lvlTemp = 0x00;
byte lvlVolCan=0x0e;

byte i = 0;
byte j = 0;
byte blnFrontCam = 1;
unsigned long timeFrontCamPressed = 0;

byte blnCamDisplayOn=0;
bool blnSub = true; //set to 1 when we need to change Bass level
bool blnSubOld = false;

bool blnSwitch = true;
bool blnSwitchOld = false;
unsigned long timeDigitalInputChanged = 0;
#define delayBeforeEnableVolByCan 3000

byte blnEncButtonState=1;
byte blnEncButtonStatePrev=1;
bool blnVolByEnc = false;
byte bytVolBy = 0;
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

#define intCANShieldInitRetry 5
#define timeButtonWaitForInput 250
#define timeButtonDebounce 30
#define timeShortButtonPress 250
#define maxLevel 255
#define minLevel 0

#define eepromAddrblnSub 0
#define eepromAddrlvlSub 1
#define eepromAddrLvlVol 2
#define eepromAddrBri 3
#define eepromAddrBarType 4
#define eepromAddrVolBy 5

//menu
bool blnMenu = false;
bool blnSubMenuActive = false;
bool blnRefreshDisplay = false;
unsigned long timeMenuEnabled = 0;
#define intMenuTimeout 7000 //mseconds
int intMenuItem = 0;
byte bytDisplayBri = 2;
#define autoOledBri 3
#define minOledBri 0
#define OledBriInterval 5000
#define GrayScaleType 1 // 0- light50% 1- dark75% 
bool blnMenuButton = false;

char menuOledDigIn[] = "Digital In";
char menuOledBri[] = "Disp. bri.";                   
char menuOledVolBarType[] = "Volume Bar Type";
char menuOledVolCtrlBy[] = "Volume Ctrl By";
char menuOledSaveDefaults[] = "Save Defaults";
char menuOledSniffingCANGear[] = "Sniffing CAN Gear";
char menuOledSniffingCANVol[] = "Sniffing CAN Vol";


char * arrMenuOled[] = {menuOledBri, menuOledVolBarType, menuOledVolCtrlBy, menuOledSaveDefaults, menuOledSniffingCANGear, menuOledSniffingCANVol};
//char * arrMenuOled[] = {menuOledBri, menuOledVolBarType, menuOledVolCtrlBy, menuOledSaveDefaults};
//char * arrMenuOled[] = {menuOledBri, menuOledVolBarType, menuOledSaveDefaults};
#define MenuOledItems 5 // starting from 0
#define MaxMenuOledItems 5 //starting from 0. Number of menu items on one screen


char menuOledSubHi[] = "Hi";
char menuOledSubMid[] = "Mid";
char menuOledSubLow[] = "Low";
char menuOledSubAuto[] = "Auto";
char * arrMenuOledSubBri[] = {menuOledSubHi, menuOledSubMid, menuOledSubLow,menuOledSubAuto};
#define MenuOledSubBriItems 3

char menuOledSubBar1[] = "Bar1"; 
char menuOledSubBar2[] = "Bar2"; 
char menuOledSubDig[] = "Dig";
char * arrMenuOledSubVolBarType[] = {menuOledSubBar1, menuOledSubBar2, menuOledSubDig};
#define MenuOledSubVolBarTypeItems 2

char menuOledCan[] = "Can";
char menuOledEnc[] = "Encoder";
char * arrMenuOledSubVolBy[] = {menuOledCan, menuOledEnc};
#define MenuOledSubVolBy 1
//int intLDRValue = 0;

volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile byte encoderPos = 0x80;
volatile byte reading = 0;

void PinA(){
  cli();
  reading = PIND & 0xC;
  if(reading == B00001100 && aFlag) { encoderPos ++; bFlag = 0; aFlag = 0;} else if (reading == B00000100) bFlag = 1;
  sei();
}

void PinB(){
  cli();
  reading = PIND & 0xC;
  if (reading == B00001100 && bFlag) { encoderPos --; bFlag = 0; aFlag = 0;} else if (reading == B00001000) aFlag = 1;
  sei();
}


//TFT display settings
#define TFT_DC    A1     // TFT DC  
#define TFT_RST   -1     // TFT RST 
#define TFT_CS    A0     // TFT CS  
//#define tftColorBack ST77XX_BLACK
#define posTFTSerial 120
//#define lblVol "Vol:"
#define lblVol "V"
#define posVolLblX 0
#define posVolLblY 0//5
#define posVolLvlX 199 //50
#define posVolLvlY 0//5
#define posVolBarX 25//0
#define posVolBarY 0//24
#define posVolBarH 24//14
#define widthVolBar 170 //240.0
#define sizeTextLbl 3 //2 each symbol =10px + 2px space; 3 15px +3px space

//#define lblBass "Bass:"
#define lblBass "B"
#define posBassLblX 0
#define posBassLblY 38//43
#define posBassLvlX 199 //62
#define posBassLvlY 38//43
#define posBassBarX 25//0
#define posBassBarY 38//43//62

#define lblDigIn "Dig In:"
#define posDigInLblX 0
#define posDigInLblY 76//82
#define posDigInLvlX 124//150
#define posDigInLvlY 76//82

uint16_t colorVolBar = ST77XX_YELLOW;
uint16_t colorBassBar = ST77XX_YELLOW;
float fltBarWidthToVolLvl = widthVolBar/256.0;
byte intVolBarWidth = 0;
float batteryVoltage =0;


INT32U canId = 0x0;
//unsigned char len = 0;
byte len = 0;
//unsigned char buf[8];
byte buf[8];
MCP_CAN CAN(pinCANShield);

bool blnSnifCANGear = false;
bool blnSnifCANVol = false;
byte bytSnifCANGear = 0x0;
byte bytSnifCANGearOld = 0x0;
byte bytSnifCANVol = 0x0;
byte bytSnifCANVolOld = 0x0;
byte bytSnifCANVol2 = 0x0;
byte bytSnifCANVolOld2 = 0x0;

bool blnUpdateCanMFD = false;


byte bytVisualType=0;

Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* standard canId, in case we need only 1(or 2 canIDs)
    CAN.init_Mask(0, 0, 0x07ff); //bin 111 1111 1111 //1fffffff   
    CAN.init_Filt(0, 0, 0x740); //canid1
    CAN.init_Filt(1, 0, 0x4c0); //canid2

    CAN.init_Mask(1, 1, 0x1fffffff); //bin 111 1111 1111
    CAN.init_Filt(2, 1, 0x17330b10);  //extcanid1
    CAN.init_Filt(3, 1, 0x17330a10);  //extcanid2
    CAN.init_Filt(4, 1, 0x17330a10);  //extcanid3
    CAN.init_Filt(5, 1, 0x17330a10);  //extcanid4

    CAN.init_Mask(0, 0, 0x0700); //bin111 0000 0000
    CAN.init_Filt(0, 0, 0x700);  //all canids 0x7**
*/

void setup() {    
  Serial.begin(115200);
  Serial.println("boot");
  pinMode(pinDigPot, OUTPUT);
  pinMode(pinDigital, OUTPUT);
  blnRefreshDisplay = true;
  //encoder start
  pinMode(pinEncA, INPUT_PULLUP); 
  pinMode(pinEncB, INPUT_PULLUP); 
  pinMode(pinEncButton, INPUT_PULLUP);
  attachInterrupt(0,PinA,RISING);
  attachInterrupt(1,PinB,RISING);
  //encoder end
  
  pinMode(pinCamButton, INPUT_PULLUP);
  pinMode(pinCamRelay, OUTPUT);

  digitalWrite(pinDigital, blnSwitch);
  digitalWrite(pinCamRelay, blnFrontCam);
  //digitalWrite(pinEncA, HIGH);
  //digitalWrite(pinEncB, HIGH);
  //attachInterrupt(0,int0,CHANGE);
  //attachInterrupt(1,int1,CHANGE);

  blnSub = EEPROM.read(eepromAddrblnSub);
  lvlSub = EEPROM.read(eepromAddrlvlSub);
  if (lvlSub > maxLevel) lvlSub=80;
  lvlVol = EEPROM.read(eepromAddrLvlVol);
  if (lvlVol > maxLevel) lvlVol=60;
  bytDisplayBri = EEPROM.read(eepromAddrBri);
  if (bytDisplayBri > MenuOledSubBriItems) bytDisplayBri=3;
  bytVisualType = EEPROM.read(eepromAddrBarType);
  if (bytVisualType>MenuOledSubVolBarTypeItems) {bytVisualType=0;}
  bytVolBy = EEPROM.read(eepromAddrVolBy);
  if (bytVolBy>MenuOledSubVolBy) {bytVolBy=0;}
  

  SPI.begin();
  digitalPotWrite(lvlVol,addrVol); //set wiper to default
  digitalPotWrite(lvlSub,addrBass); //set wiper to default
  delay(100);
  display.init(240, 240);    //display.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
  display.setRotation(2);// if the screen is flipped, remove this command

  //tftSerialPrint("boot");
  START_CANBUS_INIT:
  if(CAN_OK == CAN.begin(CAN_500KBPS,MCP_8MHz)) {
    tftSerialPrint("CAN OK");
    delay(100);
    CAN.init_Mask(1, 1, 0x1fffffff);
    CAN.init_Filt(2, 1, 0x17333110); //volume from head unit
    CAN.init_Filt(3, 1, 0x17330b00); //cam display?
    //CAN.init_Mask(0, 0, 0x07ff); 
    //CAN.init_Filt(0, 0, 0x05bf); //wheelbuttons
    //CAN.init_Filt(1, 0, 0x03dc);  //gears selector
    CAN.init_Mask(0, 0, 0x0000); 
    //0x0663 bit5 XX*256+5 -voltage
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

//void DisplayVolume(byte lvlVol, byte bytGage,byte dispType) {
//}

void DisplayMenuOled(byte selItem, bool menuFullRefresh) {

  if (menuFullRefresh) {
    display.fillScreen(ST77XX_BLACK); //tft
    display.setTextColor(ST77XX_WHITE,ST77XX_BLACK); //tft
    display.setTextSize(2); //tft + oled
    display.setCursor(0,0); //tft + oled
    if (selItem <= MaxMenuOledItems){
      j = 0;
    } else {
      j = (selItem >= MenuOledItems)?(MenuOledItems-MaxMenuOledItems):(selItem-1);
    }
    for (i=j;i<=j+MaxMenuOledItems;i=i+1){
      display.print("  ");
      display.println(arrMenuOled[i]);  
    }  
  } else {
    display.fillRect(0, 0, 12, 240, ST77XX_BLACK);    
  }
    display.fillRect(3, (selItem*16)+2, 8, 4, ST77XX_WHITE);      
}


void DisplayLevelsOled() {
  display.fillScreen(ST77XX_BLACK); //tft
  display.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
  display.setTextSize(sizeTextLbl);
  display.setCursor(posVolLblX, posVolLblY);
  display.print(lblVol);
  display.setCursor(posBassLblX, posBassLblY);
  display.print(lblBass);
  display.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
  display.setCursor(posDigInLblX, posDigInLblY);
  display.print(lblDigIn);
  display.setTextColor(ST77XX_GREEN,ST77XX_BLACK);

    if (!blnSub) {
      colorVolBar = ST77XX_YELLOW;
      colorBassBar = ST77XX_LGRAY;
    } else {
      colorVolBar = ST77XX_LGRAY;
      colorBassBar = ST77XX_YELLOW;
    }


  //display.clearDisplay(); //oled
  //tftVolBarPrint(lvlVol, 0, colorVolBar);
  tftVolBarPrint(lvlVolOld, lvlVol, 0, colorVolBar);
  //tftVolBarPrint(lvlSub, 1, colorBassBar);
  tftVolBarPrint(lvlSubOld, lvlSub, 1, colorBassBar);
  tftVolPrint(blnSwitch,2);

  //DisplayVolume(lvlSub,1,bytVisualType);
  //DisplayVolume(lvlVol,0,bytVisualType);
  //DisplayVolume(blnSwitch,2,bytVisualType);  
}

void tftSerialPrint(const char *strOut) {
  tftSerialPrint(strOut, true);
}

void tftSerialPrint(const char *strOut, bool blnLN) {
  if ((display.getCursorY() > 230)||(display.getCursorY() < posTFTSerial)) {
    display.setCursor(0, posTFTSerial);
    display.fillRect(0, posTFTSerial, 240, 240, ST77XX_BLACK);    
  }
  display.setTextSize(1);
  display.setTextColor(ST77XX_GREEN,ST77XX_BLACK);
  if (blnLN) {
    display.println(strOut);  
  } else {
    display.print(strOut);
  }
}

void tftSerialPrint(String strOut, bool blnLN) {
  if ((display.getCursorY() > 230)||(display.getCursorY() < posTFTSerial)) {
    display.setCursor(0, posTFTSerial);
    display.fillRect(0, posTFTSerial, 240, 240, ST77XX_BLACK);    
  }
  display.setTextSize(1);
  display.setTextColor(ST77XX_GREEN,ST77XX_BLACK);
  if (blnLN) {
    display.println(strOut);  
  } else {
    display.print(strOut);
  }
}


void tftVolPrint(byte Level, byte pos){
  byte oldX = display.getCursorX();
  byte oldY = display.getCursorY();
  byte bytDisplayLevel = (Level/2.56);  
  Serial.println(Level,DEC);
  Serial.println(bytDisplayLevel,DEC);
  //display.setTextSize(2); //each symbol =10px + 2px space
  display.setTextSize(sizeTextLbl); 
  display.setTextColor(ST77XX_GREEN,ST77XX_BLACK);

  switch (pos) {
  case 0: //Volume
    display.setCursor(posVolLvlX, posVolLvlY);
    break;
  case 1: //Bass Level
    display.setCursor(posBassLvlX, posBassLvlY);
    break;
  case 2: //Digital input
    display.setCursor(posDigInLvlX, posDigInLvlY);
    if (Level == 0) {
      display.print("On ");
    } else {
      display.print("Off");
    }
    break;
  }
  if (pos < 2) {
    //if (Level < 10) {
    if (bytDisplayLevel < 10) {
      //display.print("  ");
      display.print(" ");
    //} else if (Level < 100) {
    } //else if (bytDisplayLevel < 100) {
      //display.print(" ");
    //} 
    //display.print(Level);
    display.print(bytDisplayLevel);
  }
  display.setCursor(oldX, oldY);
}

void tftVolBarPrint(byte lvlVolOld, byte lvlVol, byte lvlType, uint16_t intColor){
//void tftVolBarPrint(byte lvlVol, byte lvlType, uint16_t intColor){
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
      //display.fillRect(intVolBarWidth, posY, display.width()-intVolBarWidth, posVolBarH, ST77XX_BLACK);
      display.fillRect(posX+intVolBarWidth, posY, widthVolBar-intVolBarWidth, posVolBarH, ST77XX_BLACK);
      
      //display.fillRect(posX, posY, display.width(), posVolBarH, ST77XX_BLACK);
  } else {
      display.fillRect(posX, posY, intVolBarWidth, posVolBarH, intColor);
  } 
}

void writeToCan(INT32U canId, byte len, byte b0, byte b1,byte b2, byte b3,byte b4, byte b5,byte b6, byte b7) {
  byte buf[8];
    buf[0]=b0; buf[1]=b1; buf[2]=b2; buf[3]=b3; buf[4]=b4; buf[5]=b5; buf[6]=b6; buf[7]=b7;
    CAN.sendMsgBuf(canId, 1, len, buf);
}

void loop(){  
  timeCurrent = millis();
  if (lvlVolOld != lvlVol) {
    digitalPotWrite(lvlVol,addrVol);
    tftVolBarPrint(lvlVolOld, lvlVol, 0, colorVolBar);
    lvlVolOld = lvlVol;
    blnUpdateCanMFD = true;
  }
  if (lvlSubOld != lvlSub) {
    digitalPotWrite(lvlSub,addrBass);
    tftVolBarPrint(lvlSubOld, lvlSub, 1, colorBassBar);
    lvlSubOld = lvlSub;
    blnUpdateCanMFD = true;
  }
  if (blnSwitch != blnSwitchOld) {
    tftVolPrint(blnSwitch,2);
    digitalWrite(pinDigital, blnSwitch);
    blnSwitchOld = blnSwitch;
    timeDigitalInputChanged = timeCurrent;
    blnUpdateCanMFD = true;
  }
  if (blnSubOld != blnSub) {
    if (!blnSub) {
      colorVolBar = ST77XX_YELLOW;
      colorBassBar = ST77XX_LGRAY;
    } else {
      colorVolBar = ST77XX_LGRAY;
      colorBassBar = ST77XX_YELLOW;
    }
    tftVolBarPrint(lvlVolOld, lvlVol, 0, colorVolBar);
    tftVolBarPrint(lvlSubOld, lvlSub, 1, colorBassBar);
    blnSubOld = blnSub;
  }
  
  if (blnUpdateCanMFD) {
    writeToCan(canidMFD,8,0x4C,0x50,0x0A,0x05,0x00,0x07,0x00,0x00); //showing music logo
    writeToCan(canidMFD,8,0x80,0x35,0x4c,0x55,0x1a,0x56,0x6f,0x6c);
    writeToCan(canidMFD,8,0xc0,0x3a,0x20,splitDigit(lvlVol,0),splitDigit(lvlVol,1),0x20,0x53,0x75);
    writeToCan(canidMFD,8,0xc1,0x62,0x3a,0x20,splitDigit(lvlSub,0),splitDigit(lvlSub,1),0x20,0x44);
    writeToCan(canidMFD,8,0xc2,0x69,0x67,0x49,0x6e,0x3a,0x20,0x4f);
    if (blnSwitch) {
      writeToCan(canidMFD,8,0xc3,0x6e,0x20,0x48,0x00,0x00,0x0a,0x48);
    } else {
      writeToCan(canidMFD,8,0xc3,0x66,0x66,0x48,0x00,0x00,0x0a,0x48);
    }
    writeToCan(canidMFD,8,0xc4,0x65,0x6c,0x69,0x78,0x20,0x4c,0x69);
    //writeToCan(canidMFD,8,0xc5,0x74,0x65,0x49,0x03,0x76,0x2e,0x36); //v.6
    //batteryVoltage
    writeToCan(canidMFD,8,0xc5,0x74,0x65,0x49,0x03,splitDigit(batteryVoltage,0),splitDigit(batteryVoltage,1),0x76); //12v
    writeToCan(canidMFD,8,0xc6,0x4a,0x00,0x00,0x01,0x00,0x00,0x00);
    blnUpdateCanMFD = false;
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

  if (blnMenu) {
    if (blnMenuButton) {
      blnRefreshDisplay = true;
      if (blnSubMenuActive) {blnMenu=false;}
      if (arrMenuOled[intMenuItem] == menuOledBri) {
        if (blnSubMenuActive) {
          //Serial.println("Save brightness");
          EEPROM.update(eepromAddrBri, bytDisplayBri);
        }
      }
      if (arrMenuOled[intMenuItem] == menuOledVolBarType) {
        if (blnSubMenuActive) {
          //Serial.println("Save vol bar type");
          //EEPROM.update(eepromAddrBarType, bytVisualType);
        }
      }
      if (arrMenuOled[intMenuItem] == menuOledVolCtrlBy) {
        if (!blnVolByEnc) {
          tftSerialPrint("Vol by encoder");
        } else {
          tftSerialPrint("Vol by CAN");
        }
        blnVolByEnc = !blnVolByEnc;
      }
      if (arrMenuOled[intMenuItem] == menuOledSaveDefaults) {
          //Serial.println("Save all defaults");
          EEPROM.update(eepromAddrblnSub, blnSub);
          EEPROM.update(eepromAddrlvlSub, lvlSub);
          EEPROM.update(eepromAddrLvlVol, lvlVol);
          EEPROM.update(eepromAddrVolBy, bytVolBy);
          blnMenu = false;
      }

      if (arrMenuOled[intMenuItem] == menuOledSniffingCANGear) {
        tftSerialPrint("can gear snif",true);
        blnSnifCANGear = !blnSnifCANGear;
        blnMenu = false;
      }

      if (arrMenuOled[intMenuItem] == menuOledSniffingCANVol) {
        tftSerialPrint("can vol snif",true);
        blnSnifCANVol = !blnSnifCANVol;
        blnMenu = false;
      }
      
      //blnSubMenuActive = !blnSubMenuActive;
      blnMenuButton = false;
    }

    if ((timeCurrent - timeMenuEnabled) > intMenuTimeout) { //Exit Menu By Timeout
      blnMenu = false;
      blnSubMenuActive = false;
      intMenuItem = 0;
      blnRefreshDisplay = true;
    }
  }

  if (blnRefreshDisplay) {
    if (blnMenu) {
      if (blnSubMenuActive){
        //if (arrMenuOled[intMenuItem] == menuOledBri) {       DisplaySubMenuOled(bytDisplayBri,0);}
        //if (arrMenuOled[intMenuItem] == menuOledVolBarType) {DisplaySubMenuOled(bytVisualType,1);}
        //if (arrMenuOled[intMenuItem] == menuOledVolCtrlBy) { DisplaySubMenuOled(bytVolBy     ,2);}    
      } else {
        DisplayMenuOled(intMenuItem, true);
        //tmTextPrint(arrMenuTM[intMenuItem]);
      }
    } else {
      DisplayLevelsOled();
    }
    blnRefreshDisplay = false;
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

  //if ((buttonPressedCount > 0)&&((timeCurrent-timeButtonPressedLastTime) > timeButtonWaitForInput)) {
  if ((buttonPressedCount>=2)||((buttonPressedCount > 0)&&((timeCurrent-timeButtonPressedLastTime) > timeButtonWaitForInput))) {
      if (buttonPressedCount == 1) {
        if (timeButtonPressedDuration < timeShortButtonPress) { //short enc button press
          if (blnMenu) {
            blnMenuButton = true;
          } else {
            //blnRefreshDisplay = true;
            blnSub = !blnSub;
          }
        } else { //long enc button press
          blnMenu = true;
          blnRefreshDisplay = true;
          timeMenuEnabled = timeCurrent;
        }
      } else { //double click
        blnSwitch = !blnSwitch;
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
              //tftSerialPrint("DIG IN by CAN");
              blnSwitch = !blnSwitch;
          }
        } 
        if (canId == 0x663) { //voltage //0x0663 bit5 XX*256+5 -voltage
          batteryVoltage = buf[5]*256+5;            
        }
        if (canId == 0x3dc) { //gear selector
          bytSnifCANGear = buf[5];
          if (bytSnifCANGear != bytSnifCANGearOld) {
            if (blnSnifCANGear) {
              tftSerialPrint("can id 0x3dc ",false);
              tftSerialPrint(String(buf[5],HEX),true);
            }
            if ((bytSnifCANGear & 0x0F)==0x06) {
              intGear = 255;
            } else {
              if ((bytSnifCANGear & 0x0F)>0x07) {
                intGear = 1;
              }
            }
            bytSnifCANGearOld = bytSnifCANGear;        
          } 
        }
        if ((canId == 0x17330b00)&&(buf[0] == 0x22)) { //cam display
          if (buf[2] == 0x11) {
            //tftSerialPrint("cam display enabled");
            blnCamDisplayOn=1;
          }
          if (buf[2] == 0x00) {
            //tftSerialPrint("cam display disabled");
            blnCamDisplayOn=0;
          }
        }
        if (canId == 0x17333110) {  //vol control
              if (blnSnifCANVol) {
                tftSerialPrint("0x17333110 ",false);
                tftSerialPrint(String(buf[0],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[1],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[2],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[3],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[4],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[5],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[6],HEX),false);
                tftSerialPrint(" ",false);
                tftSerialPrint(String(buf[7],HEX),true);
              }

          if (buf[1] == 0x6F) { //vol can 1
            bytSnifCANVol = buf[5];
            if (bytSnifCANVolOld != bytSnifCANVol) {
//              if (blnSnifCANVol) {
//                tftSerialPrint("canid 0x17333110 buf[1]=0x6f buf[0]=",false);
//                tftSerialPrint(String(buf[0],HEX),false);
//                tftSerialPrint(" buf[5]=",false);
//                tftSerialPrint(String(bytSnifCANVol,HEX),true);
//              }
              if ((buf[0] == 0x4C)||(buf[0] == 0x3C)) {
                lvlVolCan=bytSnifCANVol; //Vol   
              }
              bytSnifCANVolOld = bytSnifCANVol;
            }
          }
          if ((buf[1] == 0x52)&&(buf[0] == 0x3C)) { //vol can 2
            bytSnifCANVol2 = buf[2];
            if (bytSnifCANVol2 != bytSnifCANVolOld2) {
//              if (blnSnifCANVol) {
//                tftSerialPrint("canid 0x17333110 buf[0]=0x3C buf[1]=0x52 buf[2]=",false);
//                tftSerialPrint(String(bytSnifCANVol2,HEX),true);
//              }
              lvlVolCan=bytSnifCANVol2; //Vol  
              bytSnifCANVolOld2 = bytSnifCANVol2;
            }
          }
          if ((!blnSwitch)&&(!blnVolByEnc)) {
            if ((timeCurrent-timeDigitalInputChanged)>delayBeforeEnableVolByCan) {
              lvlTemp = lvlVolCan*8.5;
              if (lvlTemp != lvlVol) {
                 lvlVol = lvlTemp;
              }
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
  
  if(encoderPos != 0x80) {
    lvlTemp = (encoderPos - 0x80);
    encoderPos = 0x80;
    if (blnMenu) {
      //blnRefreshDisplay = true;
      if (blnSubMenuActive){
/*        if (arrMenuOled[intMenuItem] == menuOledBri) {
          bytDisplayBri = checkBriLvl(bytDisplayBri + lvlTemp);
          if (bytDisplayBri < 3) {
            setOledBri(bytDisplayBri);
            //display.dimLevel(bytDisplayBri); //0 1 2
          }
        }
        if (arrMenuOled[intMenuItem] == menuOledVolBarType) { bytVisualType = checkVolTypeNumber(bytVisualType + lvlTemp);}
        if (arrMenuOled[intMenuItem] == menuOledVolCtrlBy)  { bytVolBy = checkVolBy(bytVolBy + lvlTemp); }
 */
      } else {
        intMenuItem = checkMenuItems(intMenuItem + lvlTemp);    
      }
      DisplayMenuOled(intMenuItem, false);

      timeMenuEnabled = timeCurrent;
    } else {
      lvlTemp = lvlTemp*lvlStep;
      if (blnSub) {
        lvlSub = checkVolLevel(lvlSub + lvlTemp);
      } else {
        lvlVol = checkVolLevel(lvlVol + lvlTemp);
      }
    }
  }
}

byte checkVolLevel(int lvl) {
      if (lvl < minLevel) return minLevel;
      if (lvl > maxLevel) return maxLevel;
      //if (lvl > 94) return 96;      
      return lvl;
}

#if CANEnabled == 1
  byte splitDigit(byte lvl, byte low) {
    if (low) {
      if (lvl < 10 ) {
        return 0x30+lvl;
      } else {
        return 0x30+(lvl-(lvl/10)*10);
      }
    } else {
      if (lvl < 10 ) {
        return 0x30;
      } else {
        return 0x30+(lvl/10);
      }
    }
  }
#endif

byte checkMenuItems(int i) {
      if (i < 0) return 0;
      if (i > MenuOledItems) return MenuOledItems;
      return i;
}
