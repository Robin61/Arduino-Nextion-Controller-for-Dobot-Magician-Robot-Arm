#include <Pixy2.h>    //Library for Pixy2 camera module
#include <Nextion.h>  //Library for Nextion touchscreen
#include <EEPROM.h>   //for reading and writing data to and from EEPROM
#include <avr/wdt.h>  //Library for watchdog (used to reset arduino if loop takes too long, but also used as soft reset option)

//Define constants for input ports used in this project
const int led = 13;

const int signaturePart1 = 1;  //signature number chosen inPixy software for 1st color (part) signature
const int signaturePart2 = 2;  //signature number chosen inPixy software for 1st color (part) signature
const int signaturePart3 = 3;  //signature number chosen inPixy software for 1st color (part) signature
const int signatureCalib = 7;  //signature number chosen in Pixy software for the calibration dots

// variables for coordinate transformation from calibration sheet to dobot coordinates
float a1X = 0;       float b1X =0;      float c1X = 0; //X calibration dots on A4 sheet in Pixy coordinate system
float a1Y = 0;       float b1Y =0;      float c1Y = 0; //Y calibration dots on A4 sheet in Pixy coordinate system
float a2X = 0;       float b2X =0;      float c2X = 0; //X calibration dots in Dobot coordinate system
float a2Y = 0;       float b2Y =0;      float c2Y = 0; //Y calibration dots in Dobot coordinate system
float scaleX = 0;    float scaleY = 0;
float lr1 = 0;        float lr2 =0;  // length of a-b line segment (used as radius in calculation of angle
float tempFloatVar = 0; // variable used for temporarily storing value (used for swapping values of variables)
float tempFloatVarX = 0; float tempFloatVarY = 0; int tempIntVarSignature = 0;// variables used for temporarily storing value (used for swapping values of variables)

//other variables
float startPosX = 200.00;       float startPosY = 0.00;       float startPosZ = 0.00;       float startPosR = 0.00;       //Start position when powering up
float calXpos = 200.00;         float calYpos = 0.00;         float calZpos = 100.00;       float calRpos = 0.00;         //Arm position for reading calibration sheet
float dropLocation1X = 200.00;  float dropLocation1Y = 0.00;  float dropLocation1Z = 0.00;  float dropLocation1R = 0.00;  //Drop location for parts with signature 1
float dropLocation2X = 200.00;  float dropLocation2Y = 0.00;  float dropLocation2Z = 0.00;  float dropLocation2R = 0.00;  //Drop location for parts with signature 2
float dropLocation3X = 200.00;  float dropLocation3Y = 0.00;  float dropLocation3Z = 0.00;  float dropLocation3R = 0.00;  //Drop location for parts with signature 3
float currentX = startPosX;     float currentY = startPosY;   float currentZ = startPosZ;   float currentR = startPosR;   bool currentVac = false;
int endeffectorGripper = 0; // indicate type of end effector: true for gripper, false for vacuum cup

float zDownPos = 0;
float pixyPartSignature = 0; float pixyPartX = 0; float pixyPartY = 0; //part found by pixy camera
int signature1Found = 0;  // counter for how many Signature 1 dots were found
int signature2Found = 0;  // counter for how many Signature 2 dots were found
int signature3Found = 0;  // counter for how many Signature 3 dots were found
int activeSignature =0;   // signature number currently being used for pick and place
float partWidth = 0; float partHeight = 0; //dimensions of part found by pixy camera
float dobotPartX =0; float dobotPartY = 0;
float angle1 = 0;         // angle1: angle of rotation calibration sheet to pixy coordinate system
float angle2 = 0;         // angle2: angle of rotation calibration sheet to dobot coordinate system
float intermediateX = 0; float intermediateY = 0; // intermediate variable used in calculations

float moveInterval = 1.0;   //step increment in mm used for jogging
int activeAxis = 1;         //active axis for jogging or other movement commands

bool refreshScreen = true;  // variable to indicate screen items need to be (re)drawn
bool autoCycleOn = false;   //true when automatic cycle is activated

//variables used for delay timer without delay function (to not lock controls during delay)
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
long delayInterval = 1500;  // delay between moves of the robot arm to make sure all commands are processed properly
int moveStep = 0; //index number for the current movement step in automatic cycle

int eePromIntVar = 0;
float eePromFloatVar =0;

uint32_t speedRatio = 0;
uint32_t accelRatio = 0;

//variables for Nextion

String displayText = "...";
int CurrentPage = 0;

//end variables for nextion

//EEPROM address usage for this program: (used space of 4 bytes for each variable since floats use 4 bytes. Also used 4 bytes for integers for consistency in numbering, although this is not necessary for integers.
int eeAddressStartPosAvailable = 0; //Startpos avaliable? (set value of this address to 999 if the start position has been written previously to EEPROM
float eeAddressStartPosX = 4;
float eeAddressStartPosY = 8;
float eeAddressStartPosZ = 12;
float eeAddressStartPosR = 16;
int eeAddressCalPosAvailable = 20; //calibration Position available? (set value of this address to 999 when calibartion position has been written previously to EEPROM)
float eeAddressCalXpos = 24;
float eeAddressCalYpos = 28;
float eeAddressCalZpos = 32;
float eeAddressCalRpos = 36;
int eeAddressA1Available = 40; //A1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressA1x = 44;
float eeAddressA1y = 48;
int eeAddressB1Available = 52; //B1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressB1x = 56;
float eeAddressB1y = 60;
int eeAddressC1Available = 64; //C1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressC1x = 68;
float eeAddressC1y = 72;
int eeAddressA2Available = 76; //A2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressA2x = 80;
float eeAddressA2y = 84;
int eeAddressB2Available = 88; //B2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressB2x = 92;
float eeAddressB2y = 96;
int eeAddressC2Available = 100; //C2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressC2x = 104;
float eeAddressC2y = 108;
int eeAddressZdownAvailable = 112; //Z-down position (Z pos for grabbing parts) available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressZdown = 116;
int eeAddressDropLocation1Available = 120; //Drop location 1 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation1X = 124;
float eeAddressDropLocation1Y = 128;
float eeAddressDropLocation1Z = 132;
float eeAddressDropLocation1R = 136;
int eeAddressDropLocation2Available = 140; //Drop location 2 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation2X = 144;
float eeAddressDropLocation2Y = 148;
float eeAddressDropLocation2Z = 152;
float eeAddressDropLocation2R = 156;
int eeAddressDropLocation3Available = 160; //Drop location 3 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation3X = 164;
float eeAddressDropLocation3Y = 168;
float eeAddressDropLocation3Z = 172;
float eeAddressDropLocation3R = 176;
int eeAddressEndefectorAvailable = 180; //end effector type available? (set value of this address to 999 when end effector type has been written previously to EEPROM)
int eeAddressEndeffector = 184;
int eeAddressSpeedRatioAvailable = 188; //Speed ratio available? (set value of this address to 999 when speed ratio has been written previously to EEPROM)
int eeAddressSpeedRatio = 192;
int eeAddressAccelRatioAvailable = 196; //Acceleration ratio available? (set value of this address to 999 when acceleration has been written previously to EEPROM)
int eeAddressAccelRatio = 200;
int eeAddressDelayAvailable = 204; //delayInterval available? (set value of this address to 999 when delay has been written previously to EEPROM)
int eeAddressDelay = 208;

/****** Nextion Display *****************************************************************************
Declare your Nextion objects , pageid, component id., component name */

NexPage page0   = NexPage(0, 0, "page0");
NexPage page1   = NexPage(1, 0, "page1");
NexPage page2   = NexPage(2, 0, "page2");
NexPage page3   = NexPage(3, 0, "page3");
NexPage page4   = NexPage(4, 0, "page4");
NexPage page5   = NexPage(5, 0, "page5");
NexDSButton bt0 = NexDSButton(0,6,"bt0");
NexDSButton bt1 = NexDSButton(0,9,"bt1");
NexDSButton bt2 = NexDSButton(0,10,"bt2");
NexDSButton bt3 = NexDSButton(0,11,"bt3");
NexDSButton bt4 = NexDSButton(0,12,"bt4");
NexDSButton bt5 = NexDSButton(0,13,"bt5");
NexDSButton bt6 = NexDSButton(0,14,"bt6");
NexDSButton bt7 = NexDSButton(0,15,"bt7");
NexDSButton bt8 = NexDSButton(0,16,"bt8");
NexDSButton bt9 = NexDSButton(1,4,"bt9");
NexDSButton bt10 = NexDSButton(1,9,"bt10");
NexButton b0 = NexButton(0,0,"b0");
NexButton b1 = NexButton(0,1,"b1");
NexButton b2 = NexButton(0,2,"b2");
NexButton b3 = NexButton(1,1,"b3");
NexButton b4 = NexButton(1,2,"b4");
NexButton b5 = NexButton(1,3,"b5");
NexButton b7 = NexButton(0,7,"b7");
NexButton b12 = NexButton(0,8,"b12");
NexButton b20 = NexButton(2,5,"b20");
NexButton b21 = NexButton(2,6,"b21");
NexButton b22 = NexButton(2,7,"b22");
NexButton b23 = NexButton(2,8,"b23");
NexButton b24 = NexButton(2,9,"b24");
NexButton b25 = NexButton(2,10,"b25");
NexButton b26 = NexButton(3,5,"b26");
NexButton b27 = NexButton(3,6,"b27");
NexButton b28 = NexButton(3,7,"b28");
NexButton b29 = NexButton(3,8,"b29");
NexButton b30 = NexButton(3,9,"b30");
NexButton b31 = NexButton(3,10,"b31");
NexButton b32 = NexButton(3,11,"b32");
NexButton b33 = NexButton(3,12,"b33");
NexButton b34 = NexButton(3,13,"b34");
NexButton b35 = NexButton(3,14,"b35");
NexButton b36 = NexButton(3,15,"b36");
NexButton b37 = NexButton(3,16,"b37");
NexButton b38 = NexButton(3,17,"b38");
NexButton b42 = NexButton(4,8,"b42");
NexButton b43 = NexButton(2,17,"b43");
NexText t0 = NexText(0,1,"t0");
NexText t1 = NexText(0,4,"t1");
NexText t2 = NexText(0,17,"t2");
NexText t3 = NexText(0,18,"t3");
NexText t4 = NexText(0,19,"t4");
NexText t5 = NexText(1,5,"t5");
NexText t6 = NexText(1,6,"t6");
NexText t7 = NexText(1,7,"t7");
NexText t8 = NexText(1,8,"t8");
NexText t9 = NexText(2,11,"t9");
NexText t10 = NexText(2,12,"t10");
NexText t11 = NexText(4,4,"t11");
NexText t12 = NexText(4,5,"t12");
NexSlider h0 = NexSlider(4,2,"h0");
NexSlider h1 = NexSlider(4,3,"h1");
NexSlider h2 = NexSlider(4,6,"h2");
NexRadio r0 = NexRadio(2, 13, "r0");
NexRadio r1 = NexRadio(2, 14, "r1");

char buffer[100] = {0};

/*****Nextion display : Register a button object to the touch event list ***********/
NexTouch *nex_listen_list[] = {
  &page0,
  &page1,
  &page2,
  &page3,
  &page4,
  &page5,
  &bt0,
  &bt1,
  &bt2,
  &bt3,
  &bt4,
  &bt5,
  &bt6,
  &bt7,
  &bt8,
  &bt9,
  &bt10,
  &b7,
  &b12,
  &b20,
  &b21,
  &b22,
  &b23,
  &b24,
  &b25,
  &b26,
  &b27,
  &b28,
  &b29,
  &b30,
  &b31,
  &b32,
  &b33,
  &b34,
  &b35,
  &b36,
  &b37,
  &b38,
  &b42,
  &b43,
  &h0,
  &h1,
  &h2,
  &r0,
  &r1,

  NULL
};

/*********************************************************************************************************
**  Dobot Magician
*******************************************************************************************************/
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK 
/*********************************************************************************************************
** Global parameters Dobot Magician
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;
/********************************/

Pixy2 pixy;

// Page change event:
void page0PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  refreshScreen = true;
  Serial.println(F("Page 0 activated"));
}  // End of press event


// Page change event:
void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1;  // Set variable to 1 so from now on arduino knows page 1 is loaded on the display
  refreshScreen = true;
  Serial.println(F("Page 1 activated"));
}  // End of press event


// Page change event:
void page2PushCallback(void *ptr)  // If page 2 is loaded on the display, the following is going to execute:
{
  CurrentPage = 2;  // Set variable to 2 so from now on arduino knows page 2 is loaded on the display
  refreshScreen = true;

if (endeffectorGripper == 1) {
Serial2.print("r0.val="); Serial2.print(1); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff); // set radio button for gripper to on
Serial2.print("r1.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff); // set radio button for vacuum cup to off
Serial.println("refresh radio buttons (gripper selected)");
} else {
Serial2.print("r0.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff); // set radio button for gripper to off
Serial2.print("r1.val="); Serial2.print(1); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff); // set radio button for vacuum cup to on
Serial.println("refresh radio buttons (vacuum cup selected)");
}

  Serial.println(F("Page 2 activated"));
}  // End of press event

void page3PushCallback(void *ptr)  // If page 3 is loaded on the display, the following is going to execute:
{
  CurrentPage = 3;  // Set variable to 3 so from now on arduino knows page 3 is loaded on the display
  refreshScreen = true;

  Serial.println(F("Page 3 activated"));
}  // End of press event

void page4PushCallback(void *ptr)  // If page 4 is loaded on the display, the following is going to execute:
{
  CurrentPage = 4;  // Set variable to 4 so from now on arduino knows page 4 is loaded on the display
  refreshScreen = true;

Serial2.print("n0.val="); Serial2.print(speedRatio); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("h0.val="); Serial2.print(speedRatio); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("n1.val="); Serial2.print(accelRatio); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("h1.val="); Serial2.print(accelRatio); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("n2.val="); Serial2.print(delayInterval); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("h2.val="); Serial2.print(delayInterval); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
  
  Serial.println(F("Page 4 activated"));

  Serial.print(F("gPTPCommonParams.velocityRatio = ")); Serial.println(gPTPCommonParams.velocityRatio);
  Serial.print(F("gPTPCommonParams.accelerationRatio = ")); Serial.println(gPTPCommonParams.accelerationRatio);
  Serial.print(F("delayInterval = ")); Serial.println(delayInterval);
  
}  // End of press event

void page5PushCallback(void *ptr)  // If page 5 is loaded on the display, the following is going to execute:
{
  CurrentPage = 5;  // Set variable to 5 so from now on arduino knows page 5 is loaded on the display
  refreshScreen = true;
  Serial.println(F("Page 5 activated"));
}  // End of press event


void bt0PopCallback(void *ptr)
{
    uint32_t dual_state;
    NexDSButton *btn = (NexDSButton *)ptr;
   dbSerialPrintln("bt0PopCallback");
   dbSerialPrint("ptr=");
   dbSerialPrintln((uint32_t)ptr); 
    memset(buffer, 0, sizeof(buffer));

    /* Get the state value of dual state button component . */
    bt0.getValue(&dual_state);
    if(dual_state) 
    {
        moveArm(currentX, currentY, currentZ, currentR, true);
        }
    else
    {
        moveArm(currentX, currentY, currentZ, currentR, false);
    }
}

void bt1PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt1.getValue(&dual_state);
    if(dual_state) 
    {
        activeAxis = 1;
     }
    else
    { }
}

void bt2PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt2.getValue(&dual_state);
    if(dual_state) 
    {
        activeAxis = 2;
    }
    else
    { }
}
void bt3PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt3.getValue(&dual_state);
    if(dual_state) 
    {
        activeAxis = 3;
    }
    else
    { }
}
void bt4PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt4.getValue(&dual_state);
    if(dual_state) 
    {
        activeAxis = 4;
    }
    else
    { }
}

void bt5PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt5.getValue(&dual_state);
    if(dual_state) 
    {
        moveInterval = 0.1;
    }
    else
    { }
}
void bt6PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt6.getValue(&dual_state);
    if(dual_state) 
    {
        moveInterval = 1.0;
    }
    else
    { }
}

void bt7PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt7.getValue(&dual_state);
    if(dual_state) 
    {
        moveInterval = 10;
    }
    else
    { }
}
void bt8PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt8.getValue(&dual_state);
    if(dual_state) 
    {
        moveInterval = 20;
    }
    else
    { }
}

void bt9PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt9.getValue(&dual_state);
    if(dual_state) 
    {
        autoCycleOn = true;
        currentMillis = millis;
        previousMillis = currentMillis;
        moveStep = 1;
        Serial.println(F("Start Auto Cycle"));
    }
    else
    { }
}
void bt10PopCallback(void *ptr)
{
    uint32_t dual_state;
    bt10.getValue(&dual_state);
    if(dual_state) 
    {
       autoCycleOn = false;
       moveStep = 0;
       Serial.println(F("Stop Auto Cycle")); 
    }
    else
    { }
}
void b0PopCallback(void *ptr){
  Serial.println(F("Jog"));
  refreshScreen = true;
}
void b1PopCallback(void *ptr){
  Serial.println(F("Auto"));
  refreshScreen = true;
}
void b2PopCallback(void *ptr){
  Serial.println(F("Settings"));
  refreshScreen = true;
}

void b7PopCallback(void *ptr){
  if (activeAxis == 1) moveArm((gPTPCmd.x += moveInterval), currentY, currentZ, currentR, currentVac);
  if (activeAxis == 2) moveArm(currentX, (gPTPCmd.y += moveInterval), currentZ, currentR, currentVac);
  if (activeAxis == 3) moveArm(currentX, currentY, (gPTPCmd.z += moveInterval), currentR, currentVac);
  if (activeAxis == 4) moveArm(currentX, currentY, currentZ, (gPTPCmd.r += moveInterval), currentVac);
  refreshScreen = true;
}

//Button b12 component popcallback function
// When OFF button is released the LED turns OFF and the state text changes

void b12PopCallback(void *ptr){
  if (activeAxis == 1) moveArm((gPTPCmd.x -= moveInterval), currentY, currentZ, currentR, currentVac);
  if (activeAxis == 2) moveArm(currentX, (gPTPCmd.y -= moveInterval), currentZ, currentR, currentVac);
  if (activeAxis == 3) moveArm(currentX, currentY, (gPTPCmd.z -= moveInterval), currentR, currentVac);
  if (activeAxis == 4) moveArm(currentX, currentY, currentZ, (gPTPCmd.r -= moveInterval), currentVac);
  refreshScreen = true;
}
void b20PopCallback(void *ptr){ 
//save the position at which the camera can see all parts (this is also the position used for calibration routine)
calXpos = gPTPCmd.x; calYpos = gPTPCmd.y; calZpos = gPTPCmd.z; calRpos = gPTPCmd.r;

EEPROM.put(eeAddressCalXpos, calXpos); EEPROM.get(eeAddressCalXpos, calXpos); Serial.print(F("calXpos stored in eeprom: ")); Serial.println(calXpos);
EEPROM.put(eeAddressCalYpos, calYpos); EEPROM.get(eeAddressCalYpos, calYpos); Serial.print(F("calYpos stored in eeprom "));  Serial.println(calYpos);
EEPROM.put(eeAddressCalZpos, calZpos); EEPROM.get(eeAddressCalZpos, calZpos); Serial.print(F("calZpos stored in eeprom "));  Serial.println(calZpos);
EEPROM.put(eeAddressCalRpos, calRpos); EEPROM.get(eeAddressCalRpos, calRpos); Serial.print(F("calRpos stored in eeprom"));   Serial.println(calRpos);
EEPROM.put(eeAddressCalPosAvailable, 999); // setting this address to 999 means calibration position has been stored and and can be safely retreived when arduino restarts.
EEPROM.get(eeAddressCalPosAvailable, eePromIntVar); Serial.print(F("calPosAvailable stored in EEPROM: ")); Serial.println(eePromIntVar);
}

void b21PopCallback(void *ptr){
//Move arm to the position at which the camera can see all parts (this is also the position used for calibration routine)
moveArm(calXpos, calYpos, calZpos, currentR, currentVac);
}
void b22PopCallback(void *ptr){
//Save start position (initial pose when powering on Dobot)
EEPROM.put (eeAddressStartPosX, gPTPCmd.x); EEPROM.put (eeAddressStartPosY, gPTPCmd.y); EEPROM.put (eeAddressStartPosZ, gPTPCmd.z); EEPROM.put (eeAddressStartPosR, gPTPCmd.r); EEPROM.put (eeAddressStartPosAvailable, 999);
EEPROM.get(eeAddressStartPosX, startPosX); Serial.print(F("startPosX stored in eeprom: ")); Serial.println(startPosX); EEPROM.get(eeAddressStartPosY, startPosY); Serial.print(F("startPosY stored in eeprom: ")); Serial.println(startPosY);
EEPROM.get(eeAddressStartPosZ, startPosZ); Serial.print(F("startPosZ stored in eeprom: ")); Serial.println(startPosZ); EEPROM.get(eeAddressStartPosR, startPosR); Serial.print(F("startPosR stored in eeprom: ")); Serial.println(startPosR);
}
void b23PopCallback(void *ptr){
//Move arm to start position (initial pose when powering on Dobot)
moveArm(startPosX, startPosY, startPosZ, startPosR, currentVac);
}
void b24PopCallback(void *ptr){
//Save Z-Down position in EEPROM (Z-position at which parts are picked from the table)
EEPROM.put (eeAddressZdown, gPTPCmd.z); EEPROM.put (eeAddressZdownAvailable, 999); 
EEPROM.get(eeAddressZdown, zDownPos); Serial.print(F("Z-Down stored in eeprom: Z")); Serial.println(zDownPos); 
}
void b25PopCallback(void *ptr){
//Move arm to Z-Down position (Z-position at which parts are picked from the table)
moveArm(currentX, currentY, zDownPos, currentR, currentVac);
}
void b26PopCallback(void *ptr){
//Save Calibration point A to EEPROM (bottom left, origin)
EEPROM.put (eeAddressA2x, gPTPCmd.x); EEPROM.put (eeAddressA2y, gPTPCmd.y); EEPROM.put (eeAddressA2Available, 999); 
EEPROM.get(eeAddressA2x, a2X); Serial.print(F("a2X stored in eeprom: ")); Serial.println(a2X); EEPROM.get(eeAddressA2y, a2Y); Serial.print(F("a2Y stored in eeprom: ")); Serial.println(a2Y);
}
void b27PopCallback(void *ptr){
//Move arm to Calibration point A (bottom left, origin)
moveArm(a2X, a2Y, currentZ, currentR, currentVac);
}
void b28PopCallback(void *ptr){
//Save Calibration point B to EEPROM(bottom right, end of X-axis)
EEPROM.put (eeAddressB2x, gPTPCmd.x); EEPROM.put (eeAddressB2y, gPTPCmd.y); EEPROM.put (eeAddressB2Available, 999);
EEPROM.get(eeAddressB2x, b2X); Serial.print(F("b2X stored in eeprom: ")); Serial.println(b2X); EEPROM.get(eeAddressB2y, b2Y); Serial.print(F("b2Y stored in eeprom: ")); Serial.println(b2Y);
}
void b29PopCallback(void *ptr){
//Move arm to Calibration point B (bottom right, end of X-axis)
moveArm(b2X, b2Y, currentZ, currentR, currentVac);
}
void b30PopCallback(void *ptr){
//Save Calibration point C to EEPROM(Top Left, end of Y-axis)
EEPROM.put (eeAddressC2x, gPTPCmd.x); EEPROM.put (eeAddressC2y, gPTPCmd.y); EEPROM.put (eeAddressC2Available, 999);
EEPROM.get(eeAddressC2x, c2X); Serial.print(F("c2X stored in eeprom: ")); Serial.println(c2X); EEPROM.get(eeAddressC2y, c2Y); Serial.print(F("c2Y stored in eeprom: ")); Serial.println(c2Y);
}
void b31PopCallback(void *ptr){
//Move arm Calibration point C (Top Left, end of Y-axis)
moveArm(c2X, c2Y, currentZ, currentR, currentVac);
}
void b32PopCallback(void *ptr){
//Save Drop position 1 to EEPROM (location where parts with signature 1 are dropped)
EEPROM.put (eeAddressDropLocation1X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation1Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation1Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation1R , gPTPCmd.r); 
EEPROM.put (eeAddressDropLocation1Available , 999);
EEPROM.get(eeAddressDropLocation1X, dropLocation1X); Serial.print(F("Drop location1 X stored in eeprom: ")); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation1Y, dropLocation1Y); Serial.print(F("Drop location1 Y stored in eeprom: ")); Serial.println(startPosY);
EEPROM.get(eeAddressDropLocation1Z, dropLocation1Z); Serial.print(F("Drop location1 Z stored in eeprom: ")); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation1R, dropLocation1R); Serial.print(F("Drop location1 R stored in eeprom: ")); Serial.println(startPosR);

}
void b33PopCallback(void *ptr){
//Move arm to Drop position 1 (location where parts with signature 1 are dropped)
moveArm(dropLocation1X, dropLocation1Y, dropLocation1Z, dropLocation1R, currentVac);
}
void b34PopCallback(void *ptr){
//Save Drop position 2 to EEPROM (location where parts with signature 2 are dropped)
EEPROM.put (eeAddressDropLocation2X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation2Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation2Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation2R , gPTPCmd.r); 
EEPROM.put (eeAddressDropLocation2Available , 999);
EEPROM.get(eeAddressDropLocation2X, dropLocation2X); Serial.print(F("Drop location2 X stored in eeprom: ")); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation2Y, dropLocation2Y); Serial.print(F("Drop location2 Y stored in eeprom: ")); Serial.println(startPosY);
EEPROM.get(eeAddressDropLocation2Z, dropLocation2Z); Serial.print(F("Drop location2 Z stored in eeprom: ")); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation2R, dropLocation2R); Serial.print(F("Drop location2 R stored in eeprom: ")); Serial.println(startPosR);

}
void b35PopCallback(void *ptr){
//Move arm to Drop position 2 (location where parts with signature 2 are dropped)
moveArm(dropLocation2X, dropLocation2Y, dropLocation2Z, dropLocation2R, currentVac);
}
void b36PopCallback(void *ptr){
//Save Drop position 3 to EEPROM (location where parts with signature 3 are dropped)
EEPROM.put (eeAddressDropLocation3X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation3Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation3Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation3R , gPTPCmd.r); 
EEPROM.put (eeAddressDropLocation3Available , 999);
EEPROM.get(eeAddressDropLocation3X, dropLocation3X); Serial.print(F("Drop location3 X stored in eeprom: ")); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation3Y, dropLocation3Y); Serial.print(F("Drop location3 Y stored in eeprom: ")); Serial.println(startPosY);
EEPROM.get(eeAddressDropLocation3Z, dropLocation3Z); Serial.print(F("Drop location3 Z stored in eeprom: ")); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation3R, dropLocation3R); Serial.print(F("Drop location3 R stored in eeprom: ")); Serial.println(startPosR);
}
void b37PopCallback(void *ptr){
//Move arm to Drop position 3 (location where parts with signature 3 are dropped)
moveArm(dropLocation3X, dropLocation3Y, dropLocation3Z, dropLocation3R, currentVac);
}
void b38PopCallback(void *ptr){
//Calibration: Capture the 3 calibration dots and save them to EEPROM. Procedure will abort if number of found calibration dots is not equal to 3.

  static int i; static int j = 0; static int k = 0; // counters for loops
  static int CalibDotFound = 0; // counter for how many calibration dots were found
  // grab blocks!
  pixy.ccc.getBlocks();

CalibDotFound = 0;
Serial.print(F("pixy.ccc.numBlocks = ")); Serial.println(pixy.ccc.numBlocks);
for (i=0; i<pixy.ccc.numBlocks; i++){ 
  if (pixy.ccc.blocks[i].m_signature == signatureCalib){CalibDotFound++;} //check if signature is a calibration dot and add to counter if true
  pixy.ccc.blocks[i].print(); //print found signature on serial monitor
  
}
  // If there are blocks detected, print them!
  if (CalibDotFound == 3)
  {
    Serial.print(F("Detected "));
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print(F("  block "));
      Serial.print(i);
      Serial.print(F(": "));
      pixy.ccc.blocks[i].print();
      }
    //sort array with found blocks from smallest to largest
    // note that the X and Y are added for each point to easily sort without having to evaluate X and Y individually.
    // smallest value is coordinate C (top left), middle is A (bottom left) and highest number is B (bottom right).
    for (j=0; j<pixy.ccc.numBlocks; j++){
      for (k=0; k<(pixy.ccc.numBlocks-1); k++){
        if ((pixy.ccc.blocks[k].m_x + pixy.ccc.blocks[k].m_y) > (pixy.ccc.blocks[k+1].m_x + pixy.ccc.blocks[k+1].m_y)) {
         tempFloatVarX = pixy.ccc.blocks[k].m_x; tempFloatVarY = pixy.ccc.blocks[k].m_y; tempIntVarSignature = pixy.ccc.blocks[k].m_signature;
         pixy.ccc.blocks[k].m_x = pixy.ccc.blocks[k+1].m_x; pixy.ccc.blocks[k].m_y = pixy.ccc.blocks[k+1].m_y; pixy.ccc.blocks[k].m_signature = pixy.ccc.blocks[k+1].m_signature;
         pixy.ccc.blocks[k+1].m_x = tempFloatVarX; pixy.ccc.blocks[k+1].m_y = tempFloatVarY; pixy.ccc.blocks[k+1].m_signature = tempIntVarSignature;
        }
      }
    }
    
    k=0;
    for (j=0; j<pixy.ccc.numBlocks; j++){
      Serial.print(pixy.ccc.blocks[j].m_signature);
      if (pixy.ccc.blocks[j].m_signature == signatureCalib){
    //for (k=0; k<3; k++){
    //calibration point A is defined as the lower left coordinate, which has the mid value for the sum of its X and Y coordinates and therefore position 2 in the array
    if (k==1){a1X = pixy.ccc.blocks[j].m_x;} 
    if (k==1){a1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    //caliobration point B is defined as the lower right coordinate, which has the highest value for the sum of its X and Y coordinates and therefore position 3 in the array
    if (k==2){b1X =pixy.ccc.blocks[j].m_x;}
    if (k==2){b1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    //calibration point C is defined as the top left coordinate, which has the lowest value for the sum of its X and Y coordinates and therefore position 1 in the array
    if (k==0){c1X = pixy.ccc.blocks[j].m_x;}
    if (k==0){c1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    k++;
    //}
    }
    }
    //}

      Serial.println(F(" "));
      Serial.println(F("3 Calibration points found. Writing to EEPROM:"));

      EEPROM.put (eeAddressA1x, a1X); EEPROM.put (eeAddressA1y, a1Y); EEPROM.put (eeAddressA1Available, 999); 
      EEPROM.get(eeAddressA1x, a1X); Serial.print(F("a1X stored in eeprom: ")); Serial.println(a1X); EEPROM.get(eeAddressA1y, a1Y); Serial.print(F("a1Y stored in eeprom: ")); Serial.println(a1Y);

      EEPROM.put (eeAddressB1x, b1X); EEPROM.put (eeAddressB1y, b1Y); EEPROM.put (eeAddressB1Available, 999);
      EEPROM.get(eeAddressB1x, b1X); Serial.print(F("b1X stored in eeprom: ")); Serial.println(b1X); EEPROM.get(eeAddressB1y, b1Y); Serial.print(F("b1Y stored in eeprom: ")); Serial.println(b1Y);

      EEPROM.put (eeAddressC1x, c1X); EEPROM.put (eeAddressC1y, c1Y); EEPROM.put (eeAddressC1Available, 999);
      EEPROM.get(eeAddressC1x, c1X); Serial.print(F("c1X stored in eeprom: ")); Serial.println(c1X); EEPROM.get(eeAddressC1y, c1Y); Serial.print(F("c1Y stored in eeprom: ")); Serial.println(c1Y);

}  
  else{
    Serial.println(" ");
    Serial.print("Error: incorrect numer of blocks found (3 required, found: "); Serial.println(CalibDotFound);
    Serial.println("Calibration aborted");
  }
}

void b42PopCallback(void *ptr){
//Save settings for Speed ratio, Accelleration ratio and delayinterval to EEPROM

EEPROM.put (eeAddressSpeedRatio, speedRatio);     //store in EEPROM
EEPROM.put (eeAddressSpeedRatioAvailable , 999);  //setting specific address to 999 to indicate value has been written to eeprom previously
EEPROM.get (eeAddressSpeedRatio, speedRatio);     //reading back from eeprom to check if write process was ok
Serial.print(F("speedRatio stored in eeprom: ")); Serial.println(speedRatio); 

EEPROM.put (eeAddressAccelRatio, accelRatio);     //store in EEPROM
EEPROM.put (eeAddressAccelRatioAvailable  , 999); //setting specific address to 999 to indicate value has been written to eeprom previously 
EEPROM.get (eeAddressAccelRatio, accelRatio);     //reading back from eeprom to check if write process was ok
Serial.print(F("accelRatio stored in eeprom: ")); Serial.println(accelRatio); 

EEPROM.put (eeAddressDelay, delayInterval );      //store in EEPROM
EEPROM.put (eeAddressDelayAvailable  , 999);      //setting specific address to 999 to indicate value has been written to eeprom previously
EEPROM.get (eeAddressDelay, delayInterval);       //reading back from eeprom to check if write process was ok
Serial.print(F("delayInterval stored in eeprom: ")); Serial.println(delayInterval); 
}

void b43PopCallback(void *ptr){
//Reset Arduino (start program from beginning, equivalent to pressing the reset button on the board)
Serial.println(F("b43PopCallback: reboot arduino"));
wdt_disable();
wdt_enable(WDTO_250MS); // set watchdog to short time interval, which will expire quickly and trigger reboot of the arduino
delay(1000);            //long delay which will be interrupted by the expiring watchdog timer from the command above
}

void h0PopCallback(void *ptr) //release event for speed slider
{
    Serial.println(F("h0PopCallback"));
    h0.getValue(&speedRatio);
    gPTPCommonParams.velocityRatio = speedRatio;   
    Serial.print(F("gPTPCommonParams.velocityRatio = ")); Serial.println(gPTPCommonParams.velocityRatio);
    Serial2.print("n0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial2.print(speedRatio);  // This is the value you want to send to that object and atribute mentioned before.
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    SetPTPCommonParams(&gPTPCommonParams, true, &gQueuedCmdIndex);  
}

void h1PopCallback(void *ptr)  //release event for acceleration slider
{
    Serial.println(F("h1PopCallback"));
    h1.getValue(&accelRatio);
    gPTPCommonParams.accelerationRatio = accelRatio;
    Serial.print(F("gPTPCommonParams.accelerationRatio = ")); Serial.println(gPTPCommonParams.accelerationRatio);
    Serial2.print("n1.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial2.print(accelRatio);  // This is the value you want to send to that object and atribute mentioned before.
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    SetPTPCommonParams(&gPTPCommonParams, true, &gQueuedCmdIndex);
}

void h2PopCallback(void *ptr)  //release event for delayInterval slider
{
    Serial.println(F("h2PopCallback"));
    h2.getValue(&delayInterval);
    Serial.print(F("delayInterval = ")); Serial.println(delayInterval);
    Serial2.print("n2.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial2.print(delayInterval);  // This is the value you want to send to that object and atribute mentioned before.
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
}

void r0PushCallback(void *ptr)  // Press event for radio checkbox
{
 endeffectorGripper = 1;
 EEPROM.put (eeAddressEndeffector , endeffectorGripper); EEPROM.put (eeAddressEndefectorAvailable , 999);
 EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print("End effector type stored in eeprom: "); 
 Serial.print(endeffectorGripper); Serial.println("( Gripper)");  
}  // End of press event

void r1PushCallback(void *ptr)  // Press event for radio checkbox
{
 endeffectorGripper = 0;
 EEPROM.put (eeAddressEndeffector , endeffectorGripper); EEPROM.put (eeAddressEndefectorAvailable , 999);
 EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print("End effector type stored in eeprom: "); 
 Serial.print(endeffectorGripper); Serial.println("( Vacuum Cup)");  
}  // End of press event

//**************end Nextion popcallback functions************

void setup() {  

// uncomment following section to set baud rate to Nextion display (change it from the default of 9600 to something else. Only necessary if higher baud rate is needed for your project).   
/*
    Serial2.begin(9600);  //pins 16 and 17 on Arduino Mega board, with Nextion display attached.
                          //9600 is default baud rate for Nextion.
    delay(500);  // make sure port is ready for next command
    Serial2.print("bauds=115200");  //set baud rate for Nextion to 115200 permanently 
                                    //(use "baud" for temporary and "bauds" for permanent. In latter case you can remove command after executing once)
    Serial2.write(0xff);  //always send this line 3 times after each commend to Nextion display
    Serial2.write(0xff);
    Serial2.write(0xff);

// end of baud rate section
*/
  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);

// **** setup miscellaneous ********************************** 
    Serial.begin(115200);   //arduino debugging port
    delay(500);  //give serial ports time to initialize before using them
    Serial1.begin(115200);  //pins 18 and 19 on Arduino Mega board (attach Dobot Magician)
    delay(500);  //give serial ports time to initialize before using them
    Serial2.begin(115200);  //pins 16 and 17 on Arduino Mega board (attach Nextion Display)
    delay(500);  //give serial ports time to initialize before using them
    printf_begin();
    //Set Timer Interrupt
    FlexiTimer2::set(100,Serialread); 
    FlexiTimer2::start();

nexInit();  // initialize Nextion

//Nextion Display: Register the pop event callback function of the components
  page0.attachPush(page0PushCallback);  // Page press event
  page1.attachPush(page1PushCallback);  // Page press event
  page2.attachPush(page2PushCallback);  // Page press event
  page3.attachPush(page3PushCallback);  // Page press event
  page4.attachPush(page4PushCallback);  // Page press event
  page5.attachPush(page5PushCallback);  // Page press event
  bt0.attachPop(bt0PopCallback, &bt0);
  bt1.attachPop(bt1PopCallback, &bt1);
  bt2.attachPop(bt2PopCallback, &bt2);
  bt3.attachPop(bt3PopCallback, &bt3);
  bt4.attachPop(bt4PopCallback, &bt4);
  bt5.attachPop(bt5PopCallback, &bt5);
  bt6.attachPop(bt6PopCallback, &bt6);
  bt7.attachPop(bt7PopCallback, &bt7);
  bt8.attachPop(bt8PopCallback, &bt8);
  bt9.attachPop(bt9PopCallback, &bt9);
  bt10.attachPop(bt10PopCallback, &bt10);
  b0.attachPop(b0PopCallback,&b0);
  b2.attachPop(b1PopCallback,&b1);
  b2.attachPop(b2PopCallback,&b2);
  b7.attachPop(b7PopCallback,&b7);
  b12.attachPop(b12PopCallback,&b12);
  b20.attachPop(b20PopCallback,&b20);
  b21.attachPop(b21PopCallback,&b21);
  b22.attachPop(b22PopCallback,&b22);
  b23.attachPop(b23PopCallback,&b23);
  b24.attachPop(b24PopCallback,&b24);
  b25.attachPop(b25PopCallback,&b25);
  b26.attachPop(b26PopCallback,&b26);
  b27.attachPop(b27PopCallback,&b27);
  b28.attachPop(b28PopCallback,&b28);
  b29.attachPop(b29PopCallback,&b29);
  b30.attachPop(b30PopCallback,&b30);
  b31.attachPop(b31PopCallback,&b31);
  b32.attachPop(b32PopCallback,&b32);
  b33.attachPop(b33PopCallback,&b33);
  b34.attachPop(b34PopCallback,&b34);
  b35.attachPop(b35PopCallback,&b35);
  b36.attachPop(b36PopCallback,&b36);
  b37.attachPop(b37PopCallback,&b37);
  b38.attachPop(b38PopCallback,&b38);
  b42.attachPop(b42PopCallback,&b42);
  b43.attachPop(b43PopCallback,&b43);
  h0.attachPop(h0PopCallback);    // speed slider
  h1.attachPop(h1PopCallback);    // acceleration slider
  h2.attachPop(h2PopCallback);    // delay timer slider
  r0.attachPush(r0PushCallback);  // Radio checkbox Gripper Press
  r1.attachPush(r1PushCallback);  // Radio checkbox Vacuum Cup Press

Serial.println(F(" "));
Serial.println(F("===========Startup sequence, read settings from EEPROM Arduino================="));
Serial.println(F(" "));
// ********************* EEPROM load data ******************************************************
// In this section previously stored data is retrieved from the EEPROM if available. If data is not available nothing will be done or a default value will be used

EEPROM.get(eeAddressStartPosAvailable, eePromIntVar);
  if (eePromIntVar != 999) {startPosX = 200.00; startPosY = 0.00; startPosZ = 0.00; startPosR = 0.00; Serial.println(F("Start position NOT previously saved, using default values:"));
     Serial.print(F(" X")); Serial.print(startPosX); Serial.print(F(" Y")); Serial.print(startPosY); Serial.print(F(" Z")); Serial.print(startPosZ); Serial.print(F(" R")); Serial.println(startPosR);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressStartPosX, startPosX); EEPROM.get(eeAddressStartPosY, startPosY); EEPROM.get(eeAddressStartPosZ, startPosZ); EEPROM.get(eeAddressStartPosR, startPosR); 
     Serial.print(F("Start position read from EEPROM: X")); Serial.print(startPosX); Serial.print(F(" Y")); Serial.print(startPosY); Serial.print(F(" Z")); Serial.print(startPosZ); Serial.print(F(" R")); Serial.println(startPosR); }

EEPROM.get(eeAddressCalPosAvailable, eePromIntVar);
  if (eePromIntVar != 999) {calXpos = 200.00; calYpos = -45.00; calZpos = 160.00; calRpos = 0.00; Serial.println("Calibration position NOT previously saved, using default values:");
     Serial.print(F(" X")); Serial.print(calXpos); Serial.print(F(" Y")); Serial.print(calYpos); Serial.print(F(" Z")); Serial.print(calZpos); Serial.print(F(" R")); Serial.println(calRpos);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressCalXpos, calXpos); EEPROM.get(eeAddressCalYpos, calYpos); EEPROM.get(eeAddressCalZpos, calZpos); EEPROM.get(eeAddressCalRpos, calRpos); 
     Serial.print(F("Calibration position read from EEPROM: X")); Serial.print(calXpos); Serial.print(F(" Y")); Serial.print(calYpos); Serial.print(F(" Z")); Serial.print(calZpos); Serial.print(F(" R")); Serial.println(calRpos); }

EEPROM.get(eeAddressA1Available, eePromIntVar);
  if (eePromIntVar != 999) {a1X = 0.00; a1Y = 0.00; Serial.println(F("A1 calibration dot position NOT previously saved, using default values:")); Serial.print(F(" X")); Serial.print(a1X); Serial.print(F(" Y")); Serial.print(a1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressA1x, a1X); EEPROM.get(eeAddressA1y, a1Y); Serial.print(F("A1 Calibration position read from EEPROM: X")); Serial.print(a1X); Serial.print(" Y"); Serial.println(a1Y); }

EEPROM.get(eeAddressB1Available, eePromIntVar);
  if (eePromIntVar != 999) {b1X = 0.00; a1Y = 0.00; Serial.println(F("B1 calibration dot position NOT previously saved, using default values:")); Serial.print(F(" X")); Serial.print(b1X); Serial.print(F(" Y")); Serial.print(b1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressB1x, b1X); EEPROM.get(eeAddressB1y, b1Y); Serial.print(F("B1 Calibration position read from EEPROM: X")); Serial.print(b1X); Serial.print(" Y"); Serial.println(b1Y); }

EEPROM.get(eeAddressC1Available, eePromIntVar);
  if (eePromIntVar != 999) {c1X = 0.00; c1Y = 0.00; Serial.println(F("C1 calibration dot position NOT previously saved, using default values:")); Serial.print(F(" X")); Serial.print(c1X); Serial.print(F(" Y")); Serial.print(c1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressC1x, c1X); EEPROM.get(eeAddressC1y, c1Y); Serial.print(F("C1 Calibration position read from EEPROM: X")); Serial.print(c1X); Serial.print(" Y"); Serial.println(c1Y); }

EEPROM.get(eeAddressA2Available, eePromIntVar);
  if (eePromIntVar != 999) {a2X = 200.00; a2Y = -50; Serial.println(F("A2 calibration dot position NOT previously saved, using default values:")); Serial.print(F(" X")); Serial.print(a2X); Serial.print(F(" Y")); Serial.print(a2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressA2x, a2X); EEPROM.get(eeAddressA2y, a2Y); Serial.print(F("A2 Calibration position read from EEPROM: X")); Serial.print(a2X); Serial.print(" Y"); Serial.println(a2Y); }

EEPROM.get(eeAddressB2Available, eePromIntVar);
  if (eePromIntVar != 999) {b2X = 200.00; a2Y = 50; Serial.println("B2 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(b2X); Serial.print(" Y"); Serial.print(b2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressB2x, b2X); EEPROM.get(eeAddressB2y, b2Y); Serial.print(F("B2 Calibration position read from EEPROM: X")); Serial.print(b2X); Serial.print(" Y"); Serial.println(b2Y); }

EEPROM.get(eeAddressC2Available, eePromIntVar);
  if (eePromIntVar != 999) {c2X = 250.00; c2Y = -50; Serial.println("C2 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(c2X); Serial.print(" Y"); Serial.print(c2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressC2x, c2X); EEPROM.get(eeAddressC2y, c2Y); Serial.print(F("C2 Calibration position read from EEPROM: X")); Serial.print(c2X); Serial.print(" Y"); Serial.println(c2Y); }

EEPROM.get(eeAddressZdownAvailable, eePromIntVar);
  if (eePromIntVar != 999) {zDownPos = 0.00; Serial.print(F("Z-Down position NOT previously saved, using default values:")); Serial.print(F(" Z")); Serial.println(zDownPos); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressZdown, zDownPos); Serial.print(F("Z-Down position read from EEPROM: Z")); Serial.println(zDownPos); }

EEPROM.get(eeAddressDropLocation1Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation1X = 220.00; dropLocation1Y = 120.00; dropLocation1Z = 50.00; dropLocation1R = 0.00; Serial.println(F("Drop position 1 NOT previously saved, using default values:"));
     Serial.print(F(" X")); Serial.print(dropLocation1X); Serial.print(F(" Y")); Serial.print(dropLocation1Y); Serial.print(F(" Z")); Serial.print(dropLocation1Z); Serial.print(F(" R")); Serial.println(dropLocation1R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation1X, dropLocation1X); EEPROM.get(eeAddressDropLocation1Y, dropLocation1Y); EEPROM.get(eeAddressDropLocation1Z, dropLocation1Z); EEPROM.get(eeAddressDropLocation1R, dropLocation1R); 
     Serial.print(F("Drop location 1 read from EEPROM: X")); Serial.print(dropLocation1X); Serial.print(F(" Y")); Serial.print(dropLocation1Y); Serial.print(F(" Z")); Serial.print(dropLocation1Z); Serial.print(F(" R")); Serial.println(dropLocation1R); }

EEPROM.get(eeAddressDropLocation2Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation2X = 200.00; dropLocation2Y = 0.00; dropLocation2Z = 0.00; dropLocation2R = 0.00; Serial.println(F("Drop position 2 NOT previously saved, using default values:"));
     Serial.print(F(" X")); Serial.print(dropLocation2X); Serial.print(F(" Y")); Serial.print(dropLocation2Y); Serial.print(F(" Z")); Serial.print(dropLocation2Z); Serial.print(F(" R")); Serial.println(dropLocation2R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation2X, dropLocation2X); EEPROM.get(eeAddressDropLocation2Y, dropLocation2Y); EEPROM.get(eeAddressDropLocation2Z, dropLocation2Z); EEPROM.get(eeAddressDropLocation2R, dropLocation2R); 
     Serial.print(F("Drop location 2 read from EEPROM: X")); Serial.print(dropLocation2X); Serial.print(F(" Y")); Serial.print(dropLocation2Y); Serial.print(F(" Z")); Serial.print(dropLocation2Z); Serial.print(F(" R")); Serial.println(dropLocation2R); }

EEPROM.get(eeAddressDropLocation3Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation3X = 200.00; dropLocation3Y = 0.00; dropLocation3Z = 0.00; dropLocation3R = 0.00; Serial.println(F("Drop position 3 NOT previously saved, using default values:"));
     Serial.print(F(" X")); Serial.print(dropLocation3X); Serial.print(F(" Y")); Serial.print(dropLocation3Y); Serial.print(F(" Z")); Serial.print(dropLocation3Z); Serial.print(F(" R")); Serial.println(dropLocation3R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation3X, dropLocation3X); EEPROM.get(eeAddressDropLocation3Y, dropLocation3Y); EEPROM.get(eeAddressDropLocation3Z, dropLocation3Z); EEPROM.get(eeAddressDropLocation3R, dropLocation3R); 
     Serial.print(F("Drop location 3 read from EEPROM: X")); Serial.print(dropLocation3X); Serial.print(F(" Y")); Serial.print(dropLocation3Y); Serial.print(F(" Z")); Serial.print(dropLocation3Z); Serial.print(F(" R")); Serial.println(dropLocation3R); }

EEPROM.get(eeAddressEndefectorAvailable, eePromIntVar);
  if (eePromIntVar != 999) {endeffectorGripper = 0; Serial.println(F("End effector type NOT previously saved, using default: Vacuum Cup"));}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print(F("End effector type read from EEPROM: "));
      if (endeffectorGripper == 1) {Serial.println(F("Gripper"));} else Serial.println(F("Vacuum Cup"));}

EEPROM.get(eeAddressSpeedRatioAvailable , eePromIntVar);
  if (eePromIntVar != 999) {endeffectorGripper = 0; Serial.println(F("End effector type NOT previously saved, using default: Vacuum Cup"));}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print(F("End effector type read from EEPROM: "));
      if (endeffectorGripper == 1) {Serial.println(F("Gripper"));} else Serial.println(F("Vacuum Cup"));}

EEPROM.get(eeAddressSpeedRatioAvailable, eePromIntVar);
  if (eePromIntVar != 999) {speedRatio  = 50; Serial.print(F("speedRatio NOT previously saved, using default value: ")); Serial.println(speedRatio); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressSpeedRatio, speedRatio); Serial.print(F("speedRatio read from EEPROM: ")); Serial.println(speedRatio); }

EEPROM.get(eeAddressAccelRatioAvailable, eePromIntVar);
  if (eePromIntVar != 999) {accelRatio  = 50; Serial.print(F("accelRatio NOT previously saved, using default value: ")); Serial.println(accelRatio); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressAccelRatio, accelRatio); Serial.print(F("accelRatio read from EEPROM: ")); Serial.println(accelRatio); }

EEPROM.get(eeAddressDelayAvailable, eePromIntVar);
  if (eePromIntVar != 999) {delayInterval  = 1500; Serial.print(F("delayInterval NOT previously saved, using default value: ")); Serial.println(delayInterval); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDelay, delayInterval); Serial.print(F("delayInterval read from EEPROM: ")); Serial.println(delayInterval); }

// ***************** end EEPROM load data section*******************************************

// **** initialize PIXY camera *******************************
Serial.println("pixy.init");
pixy.init();
// **** setup arduino pins ***********************************

Serial2.print("bt1.val=");  // set initial state of button X
Serial2.print(1);           // this state means button is pressed
Serial2.write(0xff);         // always send these 3 lines after a writing to the display
Serial2.write(0xff);
Serial2.write(0xff);
// turn rest of axis buttons off
Serial2.print("bt2.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("bt3.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("bt4.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);

Serial2.print("bt6.val=");  // set initial state of button increment 1
Serial2.print(1);           // this state means button is pressed
Serial2.write(0xff);         // always send these 3 lines after a writing to the display
Serial2.write(0xff);
Serial2.write(0xff);
// turn rest of increment buttons off
Serial2.print("bt5.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("bt7.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);
Serial2.print("bt8.val="); Serial2.print(0); Serial2.write(0xff); Serial2.write(0xff); Serial2.write(0xff);

}


/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}

/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:    
** Output parameters:
** Returned value:      
*********************************************************************************************************/

void printf_begin(void)
{
    fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void InitRAM(void)
{
    //Set JOG Model
    gQueuedCmdIndex = 0;
    
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
    
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
 
    //Set initial pose (start position)
    moveArm(startPosX, startPosY, startPosZ, startPosR, false);
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void moveArm(float x, float y, float z, float r, bool vacuumOn)
{

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);

SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

if (endeffectorGripper == 1) {
  if (vacuumOn == false && vacuumOn != currentVac) {
      Serial.println("Open GRIPPER");
      //delay(1000);
      SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
      ProtocolProcess(); //have command(s) executed by dobot
      delay(500);
      SetEndEffectorGripper(true, true, &gQueuedCmdIndex); // open gripper (compressed air on);
      ProtocolProcess(); //have command(s) executed by dobot
      delay(500);
      SetEndEffectorGripper(false, true, &gQueuedCmdIndex);  // stop activating gripper when it is openend (compressed air off)
      delay(500); 
      }
}
else{
if (vacuumOn == false) SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
}

if (vacuumOn == true && vacuumOn != currentVac)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);

ProtocolProcess();

currentX = x;
currentY = y;
currentZ = z;
currentR = r;
currentVac = vacuumOn;
}


void loop() 
{
    
    InitRAM();

    ProtocolInit();
    
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    
    printf("\r\n======Main Program loop started======\r\n");

    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess(); 

//wdt_enable(WDTO_8S);  // set arduino watchdog timer to 8 seconds, in the main loop the timer is reset each time it passes through the loop
                      // using the wdt_reset(); command. if the reset is not done before the 8 seconds expires it probably means
                      // your program is stuck and it will reset the arduino and start the program again from the beginning.
      
    for(; ;) //start infinite loop
    {

//wdt_reset(); // reset watchdog timer. If this command is not executed before timer interval expires it assumes the arduino is frozen and will reboot the Arduino

nexLoop(nex_listen_list); // Nextion 

if (refreshScreen){
displayText = String((gPTPCmd.x),1); t1.setText(displayText.c_str());
displayText = String((gPTPCmd.y),1); t2.setText(displayText.c_str());
displayText = String((gPTPCmd.z),1); t3.setText(displayText.c_str());
displayText = String((gPTPCmd.r),1); t4.setText(displayText.c_str());
refreshScreen = false;
}

    
//Serial.print("currentMillis: "); Serial.println(currentMillis);

if (autoCycleOn = true) {
//do{
currentMillis = millis(); //used for delay function
switch (moveStep) {
  case 1:
      Serial.println("Moving to CalPos (position for detecting parts)");
      moveArm(calXpos, calYpos, calZpos, currentR, currentVac);
      previousMillis = currentMillis;
      moveStep = 2;
      break;
  case 2:
      if ((currentMillis - previousMillis) >= (4*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 3;
      }
      break;
  case 3:
      static int i; static int j = 0; static int k = 0; // counters for loops
      // grab blocks!
      pixy.ccc.getBlocks();
    
      signature1Found = 0; 
      signature2Found = 0; 
      signature3Found = 0;
      Serial.print("pixy.ccc.numBlocks = "); Serial.println(pixy.ccc.numBlocks);
      for (i=0; i<pixy.ccc.numBlocks; i++){ 
          if (pixy.ccc.blocks[i].m_signature == signaturePart1){signature1Found++; j = i; activeSignature = 1; //check if signature is a part 1 signature and set j counter to array address if true;
          pixy.ccc.blocks[i].print();} //print found signature on serial monitor
          else if (pixy.ccc.blocks[i].m_signature == signaturePart2){signature2Found++; j = i; activeSignature = 2; //check if signature is a part 1 signature and set j counter to array address if true;
          pixy.ccc.blocks[i].print();} //print found signature on serial monitor
          else if (pixy.ccc.blocks[i].m_signature == signaturePart3){signature3Found++; j = i; activeSignature = 3; //check if signature is a part 1 signature and set j counter to array address if true;
          pixy.ccc.blocks[i].print();} //print found signature on serial monitor
          }
    if (signature1Found > 0 || signature2Found > 0 || signature3Found > 0) {
      pixyPartX = pixy.ccc.blocks[j].m_x; //Select first part found in array
      pixyPartY = pixy.ccc.blocks[j].m_y;
      Serial.println("The following part will be picked:");
      pixy.ccc.blocks[j].print();
      Serial.print("pixyPartX: "); Serial.println(pixyPartX);
      Serial.print("pixyPartY: "); Serial.println(pixyPartY);
      displayText = String((pixyPartX),1); t7.setText(displayText.c_str());
      displayText = String((pixyPartY),1); t8.setText(displayText.c_str());
      //Flip Y axis (Pixy Y=0 is at the top of the image with positive direction downwards, while Dobot Y=0 is at the bottom with positive direction upwards)
      pixyPartY = (207 - pixyPartY); // Flip Y coordinate by subtracting it from max pixel number of pixy camera (207 pixels) 
      Serial.print("pixyPartY: after flipping Y axis with 207 - pixyPartY: "); Serial.println(pixyPartY);
      //calculate scale between sheet and Dobot coordinates   
      scaleX = (sqrt(sq(b2X-a2X)+sq(b2Y-a2Y)))/(sqrt(sq(b1X-a1X)+sq(b1Y-a1Y)));    Serial.print("scaleX: "); Serial.println(scaleX);
      scaleY = (sqrt(sq(c2X-a2X)+sq(c2Y-a2Y)))/(sqrt(sq(c1X-a1X)+sq(c1Y-a1Y)));    Serial.print("scaleY: "); Serial.println(scaleY);
      lr1 = (sqrt(sq(b1X-a1X)+sq(b1Y-a1Y)));     Serial.print("lr1: "); Serial.println(lr1);
      lr2 = (sqrt(sq(b2X-a2X)+sq(b2Y-a2Y)));     Serial.print("lr2: "); Serial.println(lr2);
      //translate to origin
      dobotPartX = pixyPartX - a1X;             Serial.print("dobotPartX after translate to origin: "); Serial.println(dobotPartX);       //translate to origin
      dobotPartY = pixyPartY - a1Y;             Serial.print("dobotPartY after translate to origin: "); Serial.println(dobotPartY);       //translate to origin 
      angle1     = (acos((b1X-a1X)/lr1));        Serial.print("angle1: "); Serial.println(angle1);                                    //angle between calibration sheet and pixy coordinate system
      dobotPartX = (dobotPartX*cos(-1*angle1))+(dobotPartY*sin(-1*angle1)); Serial.print("dobotPartX after rotate: "); Serial.println(dobotPartX);//rotate X coordinate from sheet to Pixy coordinate system
      dobotPartY = ((-1*dobotPartX)*sin(-1*angle1))+(dobotPartY*cos(-1*angle1));  Serial.print("dobotPartY after rotate: "); Serial.println(dobotPartY);//rotate Y coordinate from sheet to Pixy coordinate system
      dobotPartX = dobotPartX*scaleX;           Serial.print("dobotPartX after scale: "); Serial.println(dobotPartX);                //scale X between Pixy and Dobot
      dobotPartY = dobotPartY*scaleY;           Serial.print("dobotPartY after scale: "); Serial.println(dobotPartY);                //scale Y between Pixy and Dobot
      angle2     = (acos((b2X-a2X)/lr2));        Serial.print("angle2: "); Serial.println(angle2);                                    //angle between Pixy and Dobot coordinate systems
      intermediateX = dobotPartX; intermediateY = dobotPartY;               // write to intermediate variable to avoid miscalculation in next step
      dobotPartX = (intermediateX*cos(angle2))+(intermediateY*sin(angle2));       Serial.print("dobotPartX after rotate to Dobot coordinate system: "); Serial.println(dobotPartX);// rotate X coordinate to Dobot coordinate system
      dobotPartY = ((-1*intermediateX)*sin(angle2))+(intermediateY*cos(angle2));  Serial.print("dobotPartY dobotPartX after rotate to Dobot coordinate system: "); Serial.println(dobotPartY);// rotate Y coordinate to Dobot coordinate system
      dobotPartX = dobotPartX + a2X;            Serial.print("dobotPartX after translate: "); Serial.println(dobotPartX);           //translate from origin to destination
      dobotPartY = dobotPartY + a2Y;            Serial.print("dobotPartY after translate: "); Serial.println(dobotPartY);           //translate from origin to destination

      previousMillis = currentMillis;
      moveStep = 4;
      break;
      }
    else {
      autoCycleOn = false;
      moveStep = 0;
      Serial.println(" ");
      Serial.print("Error: no parts found, at least 1 part required to start pick and place action.");
      Serial.println("Pick and Place aborted");
      break;
      }
  case 4:
      Serial.println("step movestep 4, goto partXY and Zdown+20");
      moveArm(dobotPartX, dobotPartY, (zDownPos+20), currentR, currentVac);
      previousMillis = currentMillis;
      moveStep = 5;
      break;
  case 5:
      //Serial.print("currentMillis: "); Serial.print(currentMillis);Serial.print(" previousMillis: ");Serial.println(previousMillis);
      if ((currentMillis - previousMillis) >= (2*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 6;
      }
      break;
     
  case 6:
      Serial.println("movestep 6: move to ZdownPos");
      moveArm(dobotPartX, dobotPartY, zDownPos, currentR, currentVac);
      previousMillis = currentMillis;
      moveStep = 7;
      break;
  case 7:
      if ((currentMillis - previousMillis) >= (2*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 8;
      }
      break;
  case 8:
      moveArm(dobotPartX, dobotPartY, zDownPos, currentR, true);
      previousMillis = currentMillis;
      moveStep = 9;
      break;
  case 9:
      if ((currentMillis - previousMillis) >= (1*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 10;
      }
      break;
  case 10:
      switch (activeSignature) {
            case 1:
              moveArm(dobotPartX, dobotPartY, dropLocation1Z, currentR, currentVac);
              moveArm(dropLocation1X, dropLocation1Y, dropLocation1Z, currentR, currentVac);
              break;
            case 2:
              moveArm(dobotPartX, dobotPartY, dropLocation2Z, currentR, currentVac);
              moveArm(dropLocation2X, dropLocation2Y, dropLocation2Z, currentR, currentVac);
              break;
            case 3:
              moveArm(dobotPartX, dobotPartY, dropLocation3Z, currentR, currentVac);
              moveArm(dropLocation3X, dropLocation3Y, dropLocation3Z, currentR, currentVac);
              break;
              }
      previousMillis = currentMillis;
      moveStep = 11;
      break;
  case 11:
      if ((currentMillis - previousMillis) >= (4*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 12;
      }
      break;
  case 12:
      moveArm(currentX, currentY, currentZ, currentR, false);
      previousMillis = currentMillis;
      moveStep = 13;
      break;
  case 13:
      if ((currentMillis - previousMillis) >= (1*delayInterval)){
      previousMillis = currentMillis;
      moveStep = 1;
      }
      break;
    
}
  
//  }while (signature1Found > 0 || signature2Found > 0 || signature3Found > 0);

}


delay(10); 

         // end infinite loop
}   
}
