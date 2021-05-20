/* 
  TLD5542-1CHARGER SKETCH 
  
  Copyright (c) 2015, Infineon Technologies AG
  All rights reserved.

  This sketch implement a lithium battery charger prototype, powered by USB-C PD power adapter for test pourpose of the TLD5542-1 only
  WARNING:
  Lithium batteries may explode if not treated correctly see Evaluation board important notices 
  
  The hardware for this sketch is the TLD5542-1CHG_USB_SHIELD, plugged on an Arduino UNO board
  The sketch communicates with the USB barrel cable replacement IC (CYPD3177)and retrieve the maximum 
  current/Voltage profile available at the USB C power supply.
  It is possible to limit the maximum input power sinked by the shield with a user parameter 
   The charger finite state machine in the main loop, provides a constant current/Consant voltage charge profile 
  to the lithium battery.
  
  Redistribution and use in source and binary forms, with or without modification,are permitted 
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  V1.00 2021/03/13 Initial release 
  V1.01 2021/04/13 Introduced IIN limiter in powerGood function
  V1.03 2021/04/13 Introduced ERR_SHIELD and check if the shiled is configured in current mode for battery charger
                   Added condition to display USB-PD power profile only if USB is present
             V0,31 Editorial changes, comment , 
*/
// ************************* LIBRARY INCLUDE *********************************
#include "TLD5542_1_Reg_Layer.h"
#include "TLD5542_1_Func_Layer.h"
#include <SPI.h>
#include <Arduino.h>
#include "OneBitDisplay.h"
#include "BitBang_I2C.h"
#include <EEPROM.h>
#include <avr/eeprom.h>
 
// ************************* HARDWARE DEFINES *********************************
// ************(SHUNT RESISTORS, VOLTAGE DIVIDERS, REFERENECE VOLTAGE) ******
#define RSHO 30           // [mOhm]output current shunt
#define RIIN 80           // [mOhm/10] E.G. 7 mOhm write 70 DEBUG:Schematic resitance is 70
#define RVOFBL 10000UL    // [Ohm]Vout ADC resistor divider, lower resistor
#define RVOFBH 100000UL   // [Ohm]Vout ADC resistor divider, upper resistor
#define RVIFBL 10000UL    // [Ohm]Vin ADC resistor divider, lower resistor
#define RVIFBH 100000UL   // [Ohm]Vin ADC resistor divider, upper resistor
#define RVFBH_CAL 100000UL// [Ohm]upper resistor divider for ADC offset calibration (providing 1/11 of VREF)
#define RVFBL_CAL 10000UL // [Ohm]lower resistor divider for ADC offset calibration (providing 1/11 of VREF)
#define REFVOLT 4096UL    // [mV]VREF voltage applied to VREF pin
#define IINQUIESCENT 30   // [mA]quiescent current, (aprox 30mA @12V) of the shield including arduino shield with OLED ON and TLD5542-1 ON (PWMI = 0)
#define IOUTMON_ZERO 200  // [mV]IOUTMON theorical voltage at zero output current

// *************************  PIN definition *********************************
#define CSN_TLD_PIN 10 // TLD5542-1 chip select
#define TLD_EN_PIN 2   // TLD5542-1 Enable pin
#define TLD_PWMI_PIN 3 // TLD5542-1 PWMI pin
#define BUZZER  5      // Buzzer piezo
#define SDA_PIN 8      // Use -1 for the Wire library default pins or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
#define SCL_PIN 9      // Use -1 for the Wire library default pins or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
#define GHRG_EN_PIN 4  // Output mosfet to the battery, set to high to enable charge and voltage reading
#define AN_VOUT_PIN A1 // Output voltage sensing on Vdivider
#define AN_VIN_PIN A2  // Input voltage sensing on Vdivider
#define AN_IOUT_PIN A3 // Output current, ADC connected to connected to TLD5542-1 IOUTMON pin (could be eventually skipped and used for NTC)
#define AN_IIN_PIN A4  // Output current, ADC connected to connected to TLD5542-1 IINMON pin
#define AN_KEY_PIN A0  // Keyboard voltage divider
#define AN_CAL A5      // ADC  offset calibration input should be VREF applied to RVFBH_CAL and RVFBL_CAL voltage divider

// System defines
#define VIN_MIN        8000 // minimum input voltage
#define VIN_MAX        30000// MAX input voltage
#define VOUT_OVERV     30000// MAX output voltage, during open load, with resistor divider on VFB RFBL=10k RFBH=220k  it would be 30,6V, use 30V as threshold to detect if no load is connected
#define PIN_MIN        10   // minimum input voltage
#define PIN_MAX        65   // MAX input voltage
#define IIN_MAX        5500 // [mA] MAX input current , HW limited to 6.2 A (peak includes ripple ) when the TLD55421 is in buck boost and to 9.3A in booost mode by the RSWCS=8mOhm on the TLD5542-1CHG_SHIELD
#define CELL_COUNT_MIN 1    // minimum input voltage
#define CELL_COUNT_MAX 6    // MAX input voltage
#define RESCAN_DELAY   20   // [ms] keyboard rescan deleay
#define REPEAT_TOUCH_MS 200 // [ms] start repeating key on a long touch after this period

// Battery parameters  defines
#define CELL_OVERVOLT     4250 // [mV]  
#define MAXCELLVOLT       4200 // [mV]
#define STORAGECELL       3800 // [mV]
#define MINCELLVOLT       3000 //minimum cell voltage used for cell count
#define V_DEAD_BATT_V     1500 // [mV]
#define DEAD_CURR_RATIO   8    // how much is the dead current compared to the nominal one
#define INCREASE_STEP     40   // steps to reach nominal current
#define CHARGE_STOP_RATIO 10   // stop charging when output current is below nominal/ratio
#define VOUT_LOW          1    //[V] output voltage when no battery is connected

#define EEPROM_INIT       0xBEEF //value used to check if user parameters has been writen at least once
// ********************  TYPE DEFINITIONS ****************

// BUZZER FREQUENCIES ENUM
// allowed frequencies [Hz] for use with specific buzzer (PS1240P02BT), at the resonance points, to achieve higher efficiency
typedef enum freqEnum 
{
  FREQ_LOW      = 1500, 
  FREQ_MID_LOW  = 2500, 
  FREQ_MID      = 4000,
  FREQ_HIGH     = 5000, 
 }frequency;

// KEY BUTTONS ENUM
// allowed frequencies [Hz] for use with specific buzzer (PS1240P02BT) to achieve higher efficiency. 
enum keyEnum 
{
  btnNONE   =0,
  btnENTER  =1, 
  btnUP     =2, 
  btnDOWN   =3,
  btnMODE   =4, 
 };

// FSM machine states enum 
typedef enum FSMstateEnum// charger finite state machine states enum
{ SELF_CHECK, 
  INIT, 
  PREQUAL, 
  CELL_CHECK, 
  FAST_CHRG, 
  TOP_OFF, 
  DONE, 
  FAULT}FSMstate;
 
// chgErrEnum: Possible error code on the charger enum
// being in different bit position it would be possible to distinguish when multiple error are happening
enum chgErrEnum
{ NO_ERROR        = 0x0000,// task, or function performed without errors
  ERR_SPI         = 0x0001,// SPI bus error        
  ERR_IN_POWER    = 0x0002,// input voltage error     
  ERR_TLD         = 0x0004,// 
  ERR_CELL_OV     = 0x0008,// CELL error overvoltage
  ERR_CELL_LOW    = 0X0010,// CELL error
  ERR_NO_BAT      = 0X0020,// No battery present (0V at the output, error not implemented yet)
  ERR_SHIELD      = 0X0040,// shield solder jumpers not configured as current regulator, check TLD5542-1CHG_SHILED manual
  ERR_GENERIC     = 0X8000 // generic error 
};    

//paramIndex enum: selectable user paramenters index, while navigating in the charger setup menu
enum paramIndex 
{ PARAM_CELL_COUNT, // how many cells has te battery
  PARAM_VCELL,      // charge voltage per cell
  PARAM_IO_NOM,     // nominal charge current (will be reduced in casae of topping charge or max input current limited)
  PARAM_PI_MAX,     // maximum charger input current 
  MAX_PARAM};

//userVarType: struct that contains all the user set variables
typedef struct {
  unsigned int IoNom; //nominal charge current
  unsigned int cellCount;
  unsigned int cellmV; // cell charge voltage
  unsigned int PiMax; // cell charge voltage
 }userVarType;

//chrgVarType: struct that contains all the charger variables
typedef struct {
  uint16        IoSet;
  uint16        IoMon;
  uint16        IiMon;
  uint16        VoMon;
  uint16        ViMon;
  uint16        PinMon;
  uint8         tldStatus;
  chgErrEnum    err;
  FSMstateEnum  state = SELF_CHECK;
  uint32_t      usbPDOcurr  =0xDEADBEEF;// dummy value
  uint16_t      USBmAMax; 
  uint16_t      USBVout; 
}chrgVarType;

// ********************  GLOBAL VARIABLE DECLARATIONS ****************
int         ADCoffs     =0; // arduino offset in mV, calculated by 
int         iInMonOffs  =0;// TLD5542-1 iin offset
int         iOutMonOffs =0 ;// TLD5542-1 iinmonitor offset
userVarType userVar{1000,   //mA default charge current
                    3,      // default 2 cell lipo
                    4100,   // default 4100mV per cell
                    30};    // [W] default 20W max pin
chrgVarType chrgVar;

// USB BCR device instance CYPD3177
BBI2C   bbi2c_CYP; // I2C instance for CYPD3177 USB BCR device
#define CYP_I2C_ADDR 0x08
#define CURRENT_PDO_ADDRH 0x10 // current PDO register address highest 8 bits
#define CURRENT_PDO_ADDRL 0x10 // current PDO register address highest 8 bits
#define BITMASK10 0x03FF

//OLED DISPLAY instance, the one in the TLD5542-1CHG_SHIELD is a 128x64, monochrome, SSD1306 controller, Example part number: MDOB128064V2V-YI 
static uint8_t *ucBackBuffer = NULL;
OBDISP         obd;
char           sTmp[32];// temporary string for OLED writing
#define RESET_PIN -1    // let OneBitDisplay figure out the display address. Set this to -1 to disable or the GPIO pin number connected to the reset line 
#define OLED_ADDR -1    // unknown OLED address, use I2C discovery
#define FLIP180    0    // don't rotate the display
#define INVERT     0    // don't invert the display
#define USE_HW_I2C 0    // Bit-Bang the I2C bus
#define MY_OLED OLED_128x64 
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// ********************  FUNCTION DECLARATIONS *********************
uint16 readVoltageDivider(int AnalogInput, uint16 rvfbl, uint16 rvfbh);
int    read_buttons();
void   setParam(int lcd_key,int paramNum );

// ********************  FUNCTION DEFINITIONs *********************
void setup() 
{
  int tmp, rc;
  uint8_t reg_addr[2]= {CURRENT_PDO_ADDRH,CURRENT_PDO_ADDRL};// USB BCR CYPD3177 Register Address of the USB-PD CURRENT_PDO register, where it is present the max current and voltage provided on the USB-PD adater(B19..B10 voltage in 50mV unit) B9..B0 Max current in 10mA units
  uint8_t reg_data[4]= {0xDE,0xAD,0xBE,0xEF};// Dummy word to be overwritten when a register is read correctly
  int     ret_read; 
  
  onSound();  // generate wake up sound
  analogReference(EXTERNAL);// set analog reference external 
    
  // Setup fo the parameters for SPI communication, set CSN and EN pins as outputs and set EN to High  
  Serial.begin(9600);                     // sets the baud rate for communication with the computer to 9600 bauds

  pinMode(CSN_TLD_PIN, OUTPUT);           // sets CSN as output
  digitalWrite(CSN_TLD_PIN, HIGH);        // prepare CSN = high o be ready for the first SPI command
  pinMode(TLD_EN_PIN, OUTPUT);            // sets EN as output
  pinMode(TLD_PWMI_PIN, OUTPUT);          // sets PWMI as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GHRG_EN_PIN, OUTPUT);
  pinMode(AN_IIN_PIN, INPUT);
  pinMode(BUZZER, OUTPUT); // Set buzzer - pin as an output

  //setup OLED: If SDA and SCL pins are specified, they would be bit-banged in software, otherwise uses standard I2C bus at 400Khz
  rc = obdI2CInit(&obd, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 800000L); 
  
  if (rc != OLED_NOT_FOUND){
    obdSetBackBuffer(&obd, ucBackBuffer);
    obdFill(&obd, 0x0, 1);//clear OLED 
    obdWriteString(&obd, 0,0,0,(char *)"    LITIX  ", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,2,(char *)"  Power Flex", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,4,(char *)"   Charger  ", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,6,(char *)"    V1.03   ", FONT_NORMAL, 0, 1); 
    delay(1000);  
  }
 
  // setup I2C bitbang library pins for CYPD3177 USB BCR chip
  bbi2c_CYP.bWire = 0; // use bit banging
  bbi2c_CYP.iSDA  = 6; // SDA on GPIO pin
  bbi2c_CYP.iSCL  = 7; // SCL on GPIO pin 
  I2CInit(&bbi2c_CYP, 100000); // SDA=pin 10, SCL=pin 11, 100K clock
  I2CReadRegister16(&bbi2c_CYP, CYP_I2C_ADDR, &reg_addr[0] , reg_data, 4); 
  chrgVar.usbPDOcurr = (unsigned long)reg_data[0]|(unsigned long)reg_data[1]<<8 | (unsigned long)reg_data[2]<<16 | (unsigned long)reg_data[3]<<24;  // combine the 4 read byte 
  chrgVar.USBmAMax = ((chrgVar.usbPDOcurr )& BITMASK10)*10;            // first 10 bits of PDO are mA expressed in [10mA]
  chrgVar.USBVout =  (((chrgVar.usbPDOcurr>>10 )& BITMASK10)*50)/1000; // second 10 bits of PDO are V expressed in [50mV]
  
  SPI.begin();   // Initializes the SPI bus for the TLD5542-1 (not really mandatory because OLED library it is intializing)
  
  calADCoffset();//calculate ADC offset based on AN_CAL expected voltage
  calIINMON_IOUTMONoffset();//calculate IINMON offset 
}

// void sound: generate a beep on the buzzer, of a reqested frequency and duration
// @param: freq, sound desired frequency, use frequency enum to be more efficent
// @param: ms, duration in ms
void sound(frequency freq,int ms)
{
  tone(BUZZER, freq); // Send 1KHz sound signal...
  delay(ms );// call delay not in power safe 
  noTone(BUZZER);// stop sound (by stopping compare module)
}

// function onSound: generates "shut down" sound withthe buzzer. E.G. use when the input voltage is too low
void offSound()
{ 
  sound(FREQ_HIGH, 60);
  delay (60);// pause between each sound
  sound(FREQ_LOW, 60);  
}
 
// function offSound: generates "start up " sound. E.G. use when self check is successful
void onSound()
{ 
  sound(FREQ_LOW, 60);
  delay (60);// pause between each sound
  sound(FREQ_HIGH, 60);  
}
 
// function keySound: generates key pressed sound
void keySound()
{ 
  sound(FREQ_MID, 60);
}
 
// function sysSound: generic other system sound
void sysSound()
{ 
  sound(FREQ_LOW, 60);
} 

// setParam: increments and decrement selected parameter calling incDec()
// @param: lcd_key UP/DOWN increase decrease parameter 
void setParam(int lcd_key, int paramNum )
{
  switch(paramNum){        
  case PARAM_IO_NOM:// mA
    userVar.IoNom = incDec(userVar.IoNom, TLD5542_1_VFHB_VFBL_REF_uV/RSHO, (TLD5542_1_VFHB_VFBL_REF_uV/RSHO)/10 ,100,  lcd_key );
    break;
  case PARAM_CELL_COUNT:// Cell Count
    userVar.cellCount = incDec(userVar.cellCount, CELL_COUNT_MAX, CELL_COUNT_MIN, 1, lcd_key );
    break;
  case PARAM_VCELL:
    userVar.cellmV = incDec(userVar.cellmV, MAXCELLVOLT, STORAGECELL,10, lcd_key );
    break;
  case PARAM_PI_MAX:
    userVar.PiMax = incDec(userVar.PiMax, PIN_MAX, PIN_MIN,5, lcd_key );
    break;
  }    
}

// showScreenSetup: Draws main screen information on OLED display highlighting selected parameter. 
// increments and decrement selected parameter by calling setParam()
// @param: lcd_key UP/DOWN increase decrease parameter MODE:change selected parameter
void showScreenSetup( int lcd_key) 
{ 
  char outstr[15];
  uint8 invert=0;
  static int paramNum = PARAM_CELL_COUNT ;
  
  obdWriteString(&obd, 0,0,0,(char*)"BATTERY CHARGER" , FONT_NORMAL, 0, 1);  
  
  invert = (paramNum == PARAM_CELL_COUNT);  // check if highlight PARAM_CELL_COUNT parameter selection
  sprintf(sTmp, " %dS ",userVar.cellCount );
  obdWriteString(&obd, 0,0,2,sTmp , FONT_NORMAL, invert, 1);  
  
  invert = (paramNum == PARAM_VCELL);
  sprintf(sTmp, "%dmV ",userVar.cellmV);
  obdWriteString(&obd, 0,-1,-1,sTmp , FONT_NORMAL, invert, 1);  // use previous cursor position
  
  invert = (paramNum == PARAM_IO_NOM);
  sprintf(sTmp, "%dmA ",userVar.IoNom );
  obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, invert, 1);  
  invert = (paramNum == PARAM_PI_MAX);
  sprintf(sTmp, "%dW ",userVar.PiMax);
  obdWriteString(&obd, 0,-1,-1,sTmp , FONT_NORMAL, invert, 1);  // use previous cursor position
 
  if(chrgVar.USBmAMax!=0){//if USB PD adapter is detected, print USB max available power
    sprintf(sTmp, "USB %dmA %dV ",chrgVar.USBmAMax,chrgVar.USBVout);
    obdWriteString(&obd, 0,0,6,sTmp , FONT_NORMAL, 0, 1);  
  }
 
  sprintf(sTmp, "%d.%02dVi %d.%02dVo  ",chrgVar.ViMon/1000,(chrgVar.ViMon%1000)/10,chrgVar.VoMon/1000,(chrgVar.VoMon%1000)/10);// show also Vout to double check before pressing start
  obdWriteString(&obd, 0,0,7,sTmp , FONT_NORMAL, 0, 1);    
  
  switch(lcd_key) //Switch that select the parameter(btnLEFT)  and modify it (with btnUP btnDOWN)
  {
    case btnMODE:// change which parameter will be modified
      paramNum++;    
      if (paramNum>=MAX_PARAM)
      paramNum=0;
      break;
    
    case btnUP:  // change the parameter value
    case btnDOWN:// change the parameter value
      setParam(lcd_key, paramNum ); 
      break;
  }           
}

// showScreenCharge: Draws screen with charge information on OLED display. 
void showScreenCharge() 
{ 
  char outstr[15];
  static int paramNum = 0 ;
  static int ioutMontmp = 0 ;
  uint16 anIinRaw;
  
  obdWriteString(&obd, 0,0,0,(char*)"  CHARGING " , FONT_NORMAL, 0, 1);  
  
  sprintf(sTmp, "%dS ",userVar.cellCount);
  obdWriteString(&obd, 0,-1,-1,sTmp , FONT_NORMAL, 0, 1);  
  
  printState(0,1);// print charger state machine state
  
  anIinRaw = readADCWithOffs(AN_IIN_PIN);
  sprintf(sTmp, "I %d.%02dV %dmA  ",chrgVar.ViMon/1000,(chrgVar.ViMon%1000)/10, chrgVar.IiMon);
  obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, 0, 1);  
  
  sprintf(sTmp, "O %d.%02dV %dmA ", chrgVar.VoMon/1000, (chrgVar.VoMon%1000)/10 , chrgVar.IoSet );
  obdWriteString(&obd, 0,0,5,sTmp , FONT_NORMAL, 0, 1);  
  
  sprintf(sTmp, "Pi%dW Iom%dmA ",chrgVar.PinMon, chrgVar.IoMon );
  obdWriteString(&obd, 0,0,7,sTmp , FONT_NORMAL, 0, 1); 
}

// print the error string message, in current given x,y position
void printError(int x, int y)
{
  switch(chrgVar.err){  
  case NO_ERROR:
    sprintf(sTmp, "NO_ERROR ");
    break;      
  case ERR_SPI:
    sprintf(sTmp, "ERR_SPI ");
    break;
  case ERR_IN_POWER:
    sprintf(sTmp, "ERR_IN_VOLTAGE");
    break;
  case ERR_TLD:
    sprintf(sTmp, "ERR_TLD ");
    break;
  case ERR_CELL_OV:
    sprintf(sTmp, "ERR_CELL_OV ");
    break;
  case ERR_CELL_LOW:
    sprintf(sTmp, "ERR_CELL_LOW ");
    break;
  case ERR_NO_BAT:
    sprintf(sTmp, "ERR_NO_BAT ");
    break;
  case ERR_SHIELD:
    sprintf(sTmp, "ERR SHIELD VREG ");
    break;
  case ERR_GENERIC:
    sprintf(sTmp, "ERR_GENERIC ");
    break;
  default:
    sprintf(sTmp, "unknown ERR");
    break;  
  }    
  obdWriteString(&obd, 0,x,y,sTmp , FONT_NORMAL, 0, 1); 
}

//printState: prints the finite state machine current state, in string message, in current given OLED x,y position
void printState(int x, int y)
{
  switch(chrgVar.state){  
  case INIT:   // "  CHARGING 2S "
    sprintf(sTmp, "     Init     ");
    break;      
  case CELL_CHECK:
    sprintf(sTmp, "  Cell check  ");
    break;
  case PREQUAL:
    sprintf(sTmp, "   Prequal    ");
    break;
  case FAST_CHRG:
    sprintf(sTmp, "  Fast charge ");
    break;
  case TOP_OFF:
    sprintf(sTmp, "   Top off    ");
    break;
  case DONE:
    sprintf(sTmp, "    Done      ");
    break;
  case FAULT:
    sprintf(sTmp, "    Fault     ");
    break;
  default:
    break;  
  }    
  obdWriteString(&obd, 0,x,y,sTmp , FONT_NORMAL, 0, 1); 
}

// cellCountCheck: check if cells count set by the user is correct with the pack voltage read a the output
// use before start charging
chgErrEnum cellCountCheck(uint16 Vpack){
  enum chgErrEnum err = NO_ERROR;
  if (Vpack < userVar.cellCount* MINCELLVOLT ){  //check if declared cell number is correct
    err = ERR_CELL_LOW; //output voltage lower than expected for the declared cell number
  }
  else
    if (Vpack>userVar.cellCount* MAXCELLVOLT ){  //check if declared cell number is correct
      err = ERR_CELL_OV; //output voltage higer than expected for the declared cell nmber
    }
    return err;
}

// cellVoltCheck: check if cells are not in overvolage or low voltage
// this function is used while charging, then the battery voltage could esceed by little amount the MAXCELLVOLT (tipically 4,2V)
// so CELL_OVERVOLT slightly higher is used in this function
// @param:Vpack, total volage of the pack
chgErrEnum cellVoltCheck(uint16 Vpack){
  enum chgErrEnum err = NO_ERROR;
  if (Vpack < userVar.cellCount* MINCELLVOLT ){  //check if declared cell number is correct
    err = ERR_CELL_LOW; //output voltage lower than expected for the declared cell number
  }
  else
    if (Vpack>userVar.cellCount* CELL_OVERVOLT ){  //check if declared cell number is correct
      err = ERR_CELL_OV; //output voltage higer than expected for the declared cell nmber
    }
    return err;
}

// incDec: increment or decrement input value wrapping around on Max val
// @param: input, input number to be incremented/decremented
// @param: limit_max, upper limit for incrementing
// @param: limit_min, lower limit for decrementing
// @param: increment, increment step
// @param: key, keyboard input:  btnUP increase , btnDWN decrease
unsigned int incDec(uint16 input,uint16 limit_max,uint16 limit_min, uint16 increment, uint16 key)
{
  if (key==btnUP )
    if (input<=limit_max-increment)
      input+=increment;
  if (key==btnDOWN )
     if(input>=limit_min+increment )
       input-=increment;
  return input;
}  

// powerGood: check if input power and current are below the max rating of the board
// @return: return true if the input power and current are below the Max limits
bool powerGood()
{
  bool powerGood = true;
  if ( chrgVar.PinMon > userVar.PiMax){ // check if maximum input power has been exceeded
    sprintf(sTmp, " -Pin limited- ",userVar.PiMax );
    powerGood = false;
  }
  else 
    if ( chrgVar.IiMon > IIN_MAX){                      //  if maximum input current has been exceeded
      sprintf(sTmp, " -Iin limited- ",userVar.PiMax );  // then display warning in the OLED
      powerGood = false;
    }
    else   
      sprintf(sTmp, "               ",userVar.PiMax );  // delete warning in the OLED
   obdWriteString(&obd, 0,0,2,sTmp , FONT_NORMAL, 0, 1);  

   return powerGood;
}

// verifyShieldJmpCurr: verify if the shield has been configured as current reglator and not as Voltage reg by applying little output current pulse
// with disconnected load (GHRG_EN_PIN=low),if the shield is configured in current regulator mode the output voltage without load will jump to the overvoltage
// while if it is configured in voltage mode, the low analog dimming will produce a low voltage
bool verifyShieldJmpCurr()
{
  digitalWrite(GHRG_EN_PIN, LOW);     //disable battery mosfet before performing Shield HW test (short current pulse in open load)
  digitalWrite(TLD_PWMI_PIN, HIGH);   // turn ON PWMI 
  TLD5542_1_analogDimmingCurrentRegulator(500,RSHO); // set current to a very low value, if the shield is configured in current regulator mode the output voltage without load will jump to the overvotlage anyway
  delay(50);                          //wait for vout to rise to the overvoltage , if the shield is correctly configured as current generator
  chrgVar.VoMon=readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH);
  digitalWrite(TLD_PWMI_PIN, LOW);    // turn OFF PWMI 
  delay(200);                         //wait for Vout to be discharged a little from the overvoltage point
  if(chrgVar.VoMon > VOUT_OVERV)        // if vout reached the overvoltage the shield is configured correctly ****
    return true;
  else
    return false;
}

// loop: MAIN LOOP
void loop() 
{ 
  static int    prevState;
  static uint16 mA_old;
  static uint16 vFinalCharge;
  static uint16 vCel;
  static uint16 charge_stop_mA;
  int lcd_key     = btnNONE;
  uint32 PinMon;
  uint16 tmp = 0;

  // update adc radings input and output
  chrgVar.VoMon = readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH); 
  chrgVar.ViMon = readVoltageDivider(AN_VIN_PIN,(uint32)RVIFBL,(uint32)RVIFBH); 
  if (digitalRead(TLD_EN_PIN ==1)){
    chrgVar.IoMon = readIoutMon();// read ioutmon only when TLD5542-1 is enabled
    chrgVar.IiMon = readIinMon();// read iinmon only when TLD5542-1 is enabled
    chrgVar.PinMon= ((uint32)chrgVar.IiMon * (uint32)chrgVar.ViMon)/1000000;
  }
  // check input voltage: set Fault if wrong voltage is applied
  if (chrgVar.ViMon < VIN_MIN || chrgVar.ViMon > VIN_MAX ){// if VIN it is not on the allowed range, sigal it and go to FAULT state 
    chrgVar.err = ERR_IN_POWER;
    chrgVar.state=FAULT;
  }
  
  lcd_key = touchSingle();  // scan keypad

  if (chrgVar.state != prevState) // every time FSM machine change state
    obdFill(&obd, 0x0, 1);        // clear display
  
  prevState = chrgVar.state;      // update FSM state history, in order to detect if state has been changed
  
  switch(chrgVar.state){
    case SELF_CHECK: // check charger HW 
      //showScreenADC();//DEBUG:  uncomment this line and showScreenADC during ADC debugging
      digitalWrite(TLD_PWMI_PIN, LOW);        // turn off PWMI 
      digitalWrite(TLD_EN_PIN, HIGH);        // turn ON the TLD5542-1
      delay(10);
      TLD5542_1_Sync_Read_Register(TLD5542_1_STANDARD_DIAGNOSIS_ADDR);
      chrgVar.tldStatus = tld5542_1.STANDARD_DIAGNOSIS.STATE;
      if(chrgVar.tldStatus!=TLD5542_1_ACTIVE ){
        obdFill(&obd, 0x0, 1);//clear OLED
        obdWriteString(&obd, 0,0,0,(char*)"TLD ERR  " , FONT_NORMAL, 0, 1);  
        chrgVar.err = ERR_TLD;
        chrgVar.state=FAULT;
        offSound();
        delay (1000);
      }
      else{ //TLD5542-1 is active so proceed with the self check
        if ( verifyShieldJmpCurr() ){  // if the shield has been configured as current reglator, then start the LED driver application
          obdFill(&obd, 0x0, 1);       //clear OLED
          digitalWrite(GHRG_EN_PIN, HIGH);  //Enable battery mosfet to measure and charge it
          chrgVar.state = INIT;
          // read user setting from EEPROM
          eeprom_read_block((void*)&tmp, (void*)0, sizeof(tmp));//read user setting intitialization keyword to see if user parameter were already stored
          if (tmp == EEPROM_INIT)// eeprom with user parameter was written at least once
            eeprom_read_block((void*)&userVar, (void*)4, sizeof(userVar));
        }else{
            chrgVar.err = ERR_SHIELD;
            chrgVar.state=FAULT;
            offSound();
        }
      }
      break;
       
    case INIT:      // shows screen setup, requesting user parameters before charging
      digitalWrite(TLD_EN_PIN, LOW);  // turn off the TLD5542-1 //DEBUG removed for calibration
      digitalWrite(TLD_PWMI_PIN, LOW);// turn off also PWMI on TLD5542-1  //DEBUG removed for calibration
      digitalWrite(GHRG_EN_PIN, HIGH);//enable battery charge and voltage reading
      showScreenSetup(lcd_key);       // request for parameters
      chrgVar.IoSet = 0;              // set the output current variable to 0
      vFinalCharge = 0;
      if ( lcd_key == btnENTER){ // if enter button is pressed, proceed with charge
        digitalWrite(GHRG_EN_PIN, HIGH);  //Enable battery mosfet to measure and charge it
        charge_stop_mA = userVar.IoNom/CHARGE_STOP_RATIO; // set stop charging curretn , usually 1/10 of nominal current
        chrgVar.state = CELL_CHECK;
        tmp = EEPROM_INIT;//set tmp to a know keyword to check if EEPROM is correctly read
        eeprom_write_block((const void*)&tmp, (void*)0, 4 );//save the know key to show EEPROM ihas been written at least once
        eeprom_write_block((const void*)&userVar, (void*)4, sizeof(userVar));//save parameter to EEPROM for next turn on
      }
      break;

    case CELL_CHECK:  // check if the cellCount is consistent with the battery pack voltage
      chrgVar.err = cellCountCheck(chrgVar.VoMon);
      if (chrgVar.err==NO_ERROR ){
        chrgVar.state = PREQUAL;
        vFinalCharge  = userVar.cellCount * userVar.cellmV;
        vCel = chrgVar.VoMon/ userVar.cellCount;
      }
      else  //output voltage lower or higher than expected for the declared cell number
        chrgVar.state = FAULT; 
      break;
        
    case PREQUAL:   // start charging with low current
      showScreenCharge();
      if (lcd_key != btnNONE)                       // interrupt if a button is presseded
        chrgVar.state = INIT;       
      digitalWrite(TLD_EN_PIN, HIGH);               // turn ON the TLD5542-1 //DEBUG removed for calibration
      delay(10);
      TLD5542_1_currentCalibrationRoutine();        // perform calibration at the battery voltage, without current
      chrgVar.IoSet= userVar.IoNom/DEAD_CURR_RATIO; // dead battery current to a small percent of the nominal current
      TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet ,RSHO); 
      digitalWrite(TLD_PWMI_PIN, HIGH);        // start TLD5542-1 switching activity
      vCel = chrgVar.VoMon / userVar.cellCount;
      chrgVar.err = cellVoltCheck(chrgVar.VoMon);//Verify if battery voltage is OK not exceeding overvoltage
      if (chrgVar.err==NO_ERROR ){               // if battery voltage is ok , start fast charge
        chrgVar.state = FAST_CHRG;
        chrgVar.IoSet = userVar.IoNom / INCREASE_STEP;               // start with minimum step current 
        TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet,RSHO); // set current to nominal one
      }
      else{// battery voltage error has been detected
        digitalWrite(TLD_EN_PIN, LOW); // turn off immediately the TLD5542-1
        chrgVar.state = FAULT; 
      }
      break;
                 
    case FAST_CHRG:  // constant current phase: charge with nominal current, eventually reduce it if maximum power is exceeded
      showScreenCharge();
      if (lcd_key != btnNONE) // interrupt if a button is presseded
        chrgVar.state = INIT;         
      chrgVar.err = cellVoltCheck(chrgVar.VoMon);//Verify if battery voltage is OK
      if (chrgVar.err==NO_ERROR ){         // if battery voltage is ok , start fast charge
        if (chrgVar.VoMon < vFinalCharge){ //  if constant voltage Phase has not been reached yet. adjust output current and remain in FAST CHARGE state
          if ( powerGood()==false ){       // if maximum input power has been exceeded
            if(chrgVar.IoSet > userVar.IoNom/INCREASE_STEP )               // if needed to avoid IoSet going negative 
               chrgVar.IoSet = chrgVar.IoSet-(userVar.IoNom/INCREASE_STEP);// limit current  according to maximum input power
           }
           else // Maximum power & current not exceeded
             if (chrgVar.IoSet < userVar.IoNom) // keep Ioset below Io Nominal
                 chrgVar.IoSet = chrgVar.IoSet+(userVar.IoNom/INCREASE_STEP); // (DEBUG: error WITH ionom 4250 i END UP ON FAST CHARGE 4450???)
             else
                chrgVar.IoSet = userVar.IoNom;
        }
        else{ // => Vout is bigger or equal to final charge
            chrgVar.state = TOP_OFF;
            sysSound();
          }
      }
      else{// battery voltage error has been detected
        digitalWrite(TLD_EN_PIN, LOW); // turn off immediately the TLD5542-1
        chrgVar.state = FAULT; 
      }
      TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet,RSHO); 
      break;

    case TOP_OFF:  // constant voltage Phase: keep charging while reducing the output current to keep voltage stable, until the output current reaches charge_stop_mA
      showScreenCharge();
      if (lcd_key != btnNONE)// interrupt if a button is presseded
         chrgVar.state = INIT;   
      chrgVar.err = cellVoltCheck(chrgVar.VoMon);//Verify if battery voltage is OK
      if (chrgVar.err==NO_ERROR ){               // if battery voltage is ok , start fast charge
        if (chrgVar.VoMon > vFinalCharge){       // battery reached target voltage=> decrease current to keep constant voltage phase
          if (chrgVar.IoSet > charge_stop_mA)
             chrgVar.IoSet = chrgVar.IoSet-(chrgVar.IoSet/INCREASE_STEP);
          else{
            sysSound();
            chrgVar.IoSet=0;
            chrgVar.state = DONE;
            obdFill(&obd, 0x0, 1);// clear display
          }
        }
      }
      else{
        digitalWrite(TLD_EN_PIN, LOW); // turn off immediately the TLD5542-1
        chrgVar.state = FAULT;
      }
      break;

    case DONE: // charge successfully performed
      digitalWrite(TLD_EN_PIN, LOW);        // turn off the TLD5542-1
      obdWriteString(&obd, 0,0,0,(char*)"CHARGE DONE" , FONT_NORMAL, 0, 1); 
      sprintf(sTmp, "%d.%02dVi %d.%02dVo  ",chrgVar.ViMon/1000,(chrgVar.ViMon%1000)/10,chrgVar.VoMon/1000,(chrgVar.VoMon%1000)/10);// show also Vout to double check before pressing start
      obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, 0, 1);    
      
      if (chrgVar.VoMon < userVar.cellCount * MINCELLVOLT ||(lcd_key != btnNONE)){ //Battery has been disconnected or it is discharging, or a button is pressed
        chrgVar.state = INIT;         
      }
      delay(100); // wait 1 sec for the output voltage to settle down in case of disconnection
      break;
      
    case FAULT:
        digitalWrite(TLD_EN_PIN, LOW);        // turn off the TLD5542-1
        digitalWrite(GHRG_EN_PIN, LOW);   //disable battery charge
        //obdFill(&obd, 0x0, 1);//clear OLED
        obdWriteString(&obd, 0,0,0,(char*)"FAULT" , FONT_NORMAL, 0, 1); 
        printError(0,2);// print text error on oled
        if( lcd_key != btnNONE)
          chrgVar.state = SELF_CHECK;
        break;
      
    default:
      break;
    }
 }

// showScreenADC: DEBUG use when testing the HW, 
// shows ADC readings, exit if a button is pressed
void showScreenADC() 
{ 
  int lcd_key;
  uint16 anVout,anVin,anIout, anIin,anKey,anCal,anIinTMP;
  uint16 anVoutRaw,anVinRaw,anIoutRaw, anIinRaw,anKeyRaw;
  static int i = 0; // counter to show ADC sample count
  
  digitalWrite(TLD_EN_PIN, HIGH);        // turn ON the TLD5542-1 to have IOUTMON
  digitalWrite(TLD_PWMI_PIN, LOW);        // keep PWMI =0
  while(1){
    // update adc radings input and output
    anVout = readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH); 
    anVin  = readVoltageDivider(AN_VIN_PIN,(uint32)RVIFBL,(uint32)RVIFBH); 
    anIout = readADCWithOffs(AN_IOUT_PIN); 
    anIin  = readADCWithOffs(AN_IIN_PIN);  
    anKey  = readADCWithOffs(AN_KEY_PIN); 
    anCal  = readADCWithOffs(AN_CAL); 
    
    chrgVar.IoMon = readIoutMon();// read ioutmon only when TLD5542-1 is enabled
    chrgVar.IiMon = readIinMon();// read iinmon only when TLD5542-1 is enabled
    
    char outstr[15];
    static int paramNum = 0 ;
    i = i+1;  // increase ADC refresh count
    
    obdWriteString(&obd, 0,0,0,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "anVout %dmV     ",anVout);
    obdWriteString(&obd, 0,0,1,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "anVin  %dmV  ",anVin);
    obdWriteString(&obd, 0,0,2,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "anKey  %dmV   ",anKey);
    obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, 0, 1);   
    sprintf(sTmp, "Io %dmV %dmA ",anIout,chrgVar.IoMon);
    obdWriteString(&obd, 0,0,4,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "Ii %dmV %dmA  ",anIin,chrgVar.IiMon);
    obdWriteString(&obd, 0,0,5,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "anCAl  %dmV  ",anCal);
    obdWriteString(&obd, 0,0,6,sTmp , FONT_NORMAL, 0, 1);  
    sprintf(sTmp, "offs   %dmV  ",ADCoffs);
    obdWriteString(&obd, 0,0,7,sTmp , FONT_NORMAL, 0, 1);  
    
    lcd_key = touchSingle();// uncomment if you want to remain in this loop until a button is pressed
    if (lcd_key != btnNONE) // exit loop if a button is pressed
      break;                
    delay(300);
  };
  digitalWrite(TLD_EN_PIN, LOW);        // turn ON the TLD5542-1 to have IOUTMON
}

// calcualte offset:
// offset is offs= ADCread - real => if offset is negative means that the ADC reads less than real value, 
// offset has to be subtracted from the read to obtain offset free reading: real = ADCread-offs
void calADCoffset()
{
  static uint32 readOffsCal,offsRef;
  int lcd_key=btnNONE;

  
  obdFill(&obd, 0x0, 1);//clear OLED
  //calibrate ADC using CSN_A5 pin
  offsRef = analogRead(AN_CAL);// dummy first ADC read
  readOffsCal = readRaw(AN_CAL);// read pure ADC value on A5
  // calcualte voltage at the pin should be (reference scaled by res divider)
  offsRef = ((uint32)REFVOLT * (uint32)RVFBL_CAL)/((uint32)RVFBH_CAL+(uint32)RVFBL_CAL); // [mV]
  //offsRef =0;//debug restore previous line
  ADCoffs =  readOffsCal - offsRef;

 // DEBUG: print calibration values 
 /* obdWriteString(&obd, 0,0,0,(char*)"calADC press key" , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "offsRef =%d  ",offsRef);
  obdWriteString(&obd, 0,0,2,sTmp , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "readOffsCal=%d  ",readOffsCal);
  obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "ADCoffs  =%d  ",ADCoffs);
  obdWriteString(&obd, 0,0,4,sTmp , FONT_NORMAL, 0, 1);  
  delay (2000);
  */
}

// calIINMON_IOUTMONoffset: calculate offset, from typical values on TLD5542-1 IINMON and IOUTMON pins
// offset = read - ideal => if offset is negative means that the ADC reads less than expected, 
// offset have to be subtracted from the  real = read-offs
void calIINMON_IOUTMONoffset()
{
  uint32 readOffs,tmp;
  int lcd_key;

  obdFill(&obd, 0x0, 1);//clear OLED

  digitalWrite(TLD_PWMI_PIN, LOW);   // disable PWMI on TLD5542-1
  digitalWrite(TLD_EN_PIN, HIGH);   // turn ON the TLD5542-1
  delay(50);                        // wait for the TLD5542-1 to power up and power consumption to stabilize

  tmp =readADCWithOffs(AN_IIN_PIN);// read  ADC value on IINMON, with ADC offset compensated 
  readOffs =((uint32)tmp*1000)/(RIIN*2);// calculate readed current from IINMON ( RIIN is in [mA/10) , since current has to be in milliampere=> to multiply by 1000)
  iInMonOffs =readOffs - IINQUIESCENT;
  
  tmp = readADCWithOffs(AN_IOUT_PIN);// read  ADC value on IINMON, with ADC offset compensated 
  iOutMonOffs = tmp - IOUTMON_ZERO;

  digitalWrite(TLD_EN_PIN, LOW);        // turn OFF the TLD5542-1
}

// read the keypad buttons on AN_KEY_PIN resistor divider
// buttons are supposed to be pressed individually (not more than one simultaneously)
// Buttons are producingproximately 5/6 steps of 5V :0,83V 1,66V 2.5V 3.33V 
int read_buttons()
{
  uint32 adc_key_mV,tmp;
  int key;

  adc_key_mV = readADCWithOffs(AN_KEY_PIN);      // read the value from the sensor
  
  // debounce loop, prevents reading on changing keypad 
  while(1){
    delay(1);
    tmp = readADCWithOffs(AN_KEY_PIN);      // store keypad ADC in temporary variable, 
    if ( (adc_key_mV >= (tmp - tmp/16 ) ) && (adc_key_mV <= (tmp + tmp/16 ) ) ) //if the value is not changed (by 1/16th) from the last read, means that it is constant
         break; //so exit the keyboard debounce loop 
    else 
      adc_key_mV = tmp;
  };

  readADCWithOffs(AN_KEY_PIN);      // read the value from the sensor

  //  Buttons are producing aproximately 1/6 steps of 5V: btnNONE=0V  btnENTER=0,83V btnUP=1,66V btnDOWN=2.5V btnMODE=3.33V 
  if (adc_key_mV < 400)  return btnNONE;  // btnNONE produced voltage is 0V
  if (adc_key_mV < 1400) return btnENTER; // btnENTER produced voltage is 830mV
  if (adc_key_mV < 2000) return btnUP;    // ...
  if (adc_key_mV < 2700) return btnDOWN;
  return btnMODE;  // Highest voltage is produce by btnMODE
}

// read touch keypad, if a key is pressed , emit a beep , wait 300ms for release
// if key remain pressed longer than REPEAT_TOUCH_MS exit returning the key that was pressed
uint16 touchSingle()
{
  uint16  scan=0,tmp;
  static  uint16 memory=0;// used to memorize last pressed button
  static  uint16 stillTouch = 0;
  
  scan = read_buttons();// read keypad

  if (scan!=btnNONE){  
    if(memory!=scan){
      keySound();
      stillTouch=0;
    } 
    while(1){ // cycle will stop when all keys are released, or after REPEAT_TOUCH_MS to repeat a key 
      stillTouch++;
      delay(RESCAN_DELAY);// count rescan delay before rescanning
      if (stillTouch > (REPEAT_TOUCH_MS/RESCAN_DELAY)){
        memory=scan;
        delay(RESCAN_DELAY);
        break;
      }
      tmp = read_buttons();// read keypad
      if (tmp==btnNONE) // check No keys are pressed
        break;   // break while (to return memorized scan)
      scan = tmp; // otherwise update scan
    }   
  }
  else{
    stillTouch=0;
    memory=0;
  }  
  return scan;// return last released pressed button
}  

// readIinMon: reads TLD5542-1 IINMON pin and calculate input current using datasheet formula
// prerequisite: call  calIINMON_IOUTMONoffset() to remove offset from IINMON read
int readIinMon() // return IIN in mA
{
  int iinMon;
      
  iinMon = readADCWithOffs(AN_IIN_PIN);
  iinMon = ((long)iinMon*1000)/(RIIN*2) - iInMonOffs;// RIIN is in milliOhm, since current has to be in milliampere , we need to multiply by 1000
  if (iinMon < 0)
     iinMon = 0;
  return iinMon;
}

// readIinMon: reads TLD5542-1 IOMON pin and calculate outout current using datasheet formula on RSHO
// prerequisite: call  calIINMON_IOUTMONoffset() to remove offset from IINMON read
// @returns: IOUTMON in mA
uint16 readIoutMon() 
{
  uint16 ioutMon;
      
  ioutMon = readADCWithOffs(AN_IOUT_PIN)-iOutMonOffs; // remove IOUTMON offsetcalculated in calIINMON_IOUTMONoffset (0A => 200mV at IOUTMON)
  if(ioutMon<200)
    ioutMon = 200; //avoid negative current reading, which could be produced by negative offset
  ioutMon = (((uint32)ioutMon-200)*1000)/(RSHO*8);// RsHUNT is in milli, since current has to be in milliampere , we need to multiply by 1000

  return ioutMon;
}

// readADCWithOffs: read raw ADC value 
// no voltage divider moltiplication applied, only oversampling
// @param:AnalogInput, which ADC input is read
// @returns: ADC value raw
uint16 readADCWithOffs(int AnalogInput)
{  
  uint32 voltage = 0;                    // sum of samples taken
  voltage=readRaw(AnalogInput)-ADCoffs;
  if (voltage <= -ADCoffs) //If offset is negative , reading can not provide values below that offset (unused analog range)
    voltage= 0;

  return voltage;
}

// readVoltageDivider: read the voltage applied to an ADC by a voltage divider
// @param:AnalogInput , at which ADC input is connected the voltage divider
// @param:rvfbl low side resistor on the voltage divider
// @param:rvfbh high side resistor on the voltage divider
// @returns: voltage applied to the voltage divider [mV]
uint16 readVoltageDivider(int AnalogInput, uint32 rvfbl,uint32 rvfbh)
{
  uint32 sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number
  uint32 voltage = 0;            // calculated voltage

  // calculate the voltage, refVolt is the calibrated reference voltage in [V]
  voltage =readRaw(AnalogInput) - ADCoffs; // 1023
  if (voltage == -ADCoffs) //If offset is negative , reading can not provide values below that offset (unused analog range)
    voltage= 0;
  voltage = (voltage * (rvfbl+rvfbh))/rvfbl;
  
  return voltage;
}

// readRaw: returns adc read in [mV], oversampling NUM_SAMPLES[16] times
// no offset removed or voltage divider, only oversampling
// @param:AnalogInput, which ADC input is read
uint16 readRaw(int AnalogInput)
{  
  uint32 sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number
  uint16 voltage = 0;            // calculated voltage
  
  #define NUM_SAMPLES 16
  while (sample_count < NUM_SAMPLES) {
    sum += analogRead(AnalogInput);
    sample_count++;
  };
  voltage = ( ((sum * REFVOLT)>>4)>>10 ); // shift by 4 if NUM_SAMPLES is 16, then shift by 10 to divide by 1024
  
  return voltage;
}

/************************************ DEBUG & DEVELOPMENT FUNCTIONS  ************************************


// loopTestTLD: DEBUG - test TLD5542-1 by
// turning it ON, reading STD diagnosis via SPI,
// then setting output current to 1A even if battery is not connected (you can use 10 ohm 10W a resistor as a load)
// and displaying IINMON, IOUTMON, VOUT,  all charge relevant parameter
// exit when a button is pressed
void loopTestTLD() 
{ 
  digitalWrite(TLD_EN_PIN, HIGH);        // turn ON the TLD5542-1
  delay(10);
  chrgVar.IoSet= 1000;
  TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet ,RSHO); // set output current to 1000mA
  digitalWrite(GHRG_EN_PIN, HIGH);                              // Enables battery mosfet to measure and charge it
  digitalWrite(TLD_PWMI_PIN, HIGH);
  
  TLD5542_1_Sync_Read_Register(TLD5542_1_STANDARD_DIAGNOSIS_ADDR);
  chrgVar.tldStatus = tld5542_1.STANDARD_DIAGNOSIS.STATE;
  
  if(chrgVar.tldStatus!=TLD5542_1_ACTIVE ){
    obdWriteString(&obd, 0,0,0,(char*)"TLD ERR " , FONT_NORMAL, 0, 1);  
    chrgVar.err = ERR_TLD;
    offSound();
  }
    
  tld5542_1.SWTMOD.S2G_OFF=1;// update global variable to disable S2G
  TLD5542_1_Sync_Write_Register(TLD5542_1_SWTMOD_ADDR); 
  
  while(1){
     // update adc radings input and output
    chrgVar.VoMon = readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH); 
    chrgVar.ViMon = readVoltageDivider(AN_VIN_PIN,(uint32)RVIFBL,(uint32)RVIFBH); 
    if (digitalRead(TLD_EN_PIN ==1)){
      chrgVar.IoMon = readIoutMon();// read ioutmon only when TLD5542-1 is enabled
      chrgVar.IiMon = readIinMon();// read iinmon only when TLD5542-1 is enabled
    }
    
    showScreenCharge();
    delay(200);
    loopTestIINMON();
    
    if (lcd_key != btnNONE) // exit loop if a button is pressed
        break;    
  }
}

//DEBUG:
void loopTestIINMON() 
{ 
  static uint16 charge_stop_mA;
  uint32 PinMon;
  
  if (digitalRead(TLD_EN_PIN ==1)){
    chrgVar.IoMon = readIoutMon();// read ioutmon only when TLD5542-1 is enabled
    chrgVar.IiMon = readIinMon();// read iinmon only when TLD5542-1 is enabled
    PinMon = ((uint32)chrgVar.IiMon * (uint32)chrgVar.ViMon)/1000000;
  }
  Serial.print(chrgVar.VoMon );
  Serial.print("V out   ");
  Serial.print(chrgVar.IoMon );
  Serial.print("mA out   ");
  Serial.print(chrgVar.ViMon );
  Serial.print("V in  ");
  Serial.print(chrgVar.IiMon );
  Serial.print("mA in  ");
  Serial.print("Pin=");
  Serial.print(PinMon );
  Serial.println("W Pin ");
}
*/
