/* 
  TLD5542-1LED DRIVER SKETCH
  
  Copyright (c) 2015, Infineon Technologies AG
  All rights reserved.

  This sketch implement an LED driver, powered by USB-C PD power adapter, or by a standard power supply
  The hardware is the TLD5542-1CHG_USB_SHIELD with OLED display, it has to be plugged on Arduino UNO board
  The sketch communicates with the USB barrel cable replacement IC (CYPD3177)and retrieve the maximum 
  current/Voltage profile available at the USB C power supply.
  it is possible to limit the maximum input power sinked by the shield with a user parameter 
  
  Redistribution and use in source and binary forms, with or without modification,are permitted 
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
  V1.00 2021/03/13 Initial release 
  V1.01 2021/04/13 introduced IIN limiter in powerGood function
  V1.02 2021/05/05 introduced shield jumpers check
  V1.03 2021/01/06 introduced INIT wait and diagnostic clear, to avoid short to ground detection at startup (if low analog dimming is applied)
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

#define RSHO   30         // [mOhm]output current shunt
#define RIIN   80         // [mOhm/10] E.G. 7.5 mOhm write 75 Schematic S05 resitance is 80mOhm
#define RVOFBL 10000UL    // [Ohm]Vout ADC resistor divider, lower resistor
#define RVOFBH 100000UL   // [Ohm]Vout ADC resistor divider, upper resistor
#define RVIFBL 10000UL    // [Ohm]Vin ADC resistor divider, lower resistor
#define RVIFBH 100000UL   // [Ohm]Vin ADC resistor divider, upper resistor
#define RVFBH_CAL 100000UL// [Ohm]upper resistor divider for ADC offset calibration (providing 1/11 of VREF)
#define RVFBL_CAL 10000UL // [Ohm]lower resistor divider for ADC offset calibration (providing 1/11 of VREF)
#define REFVOLT 4096UL    // [mV]VREF voltage applied to VREF pin
#define IINQUIESCENT 30   // [mA]quiescent current of the shield including arduino shield with OLED ON and TLD5542-1 ON (PWMI = 0)
#define IOUTMON_ZERO 200  // [mV]IOUTMON theorical voltage at zero output current

// *************************  PIN definition *********************************
#define CSN_TLD_PIN  10 // TLD5542-1 chip select
#define TLD_EN_PIN   2  // TLD5542-1 Enable pin
#define TLD_PWMI_PIN 3  // TLD5542-1 PWMI pin
#define BUZZER       5  // Buzzer piezo
#define SDA_PIN      8  // Use -1 for the Wire library default pins or specify the pin numbers to use with bit banging on any GPIO pins
#define SCL_PIN      9  // Use -1 for the Wire library default pins or specify the pin numbers to use with bit banging on any GPIO pins
#define GHRG_EN_PIN  4  // Output mosfet to the battery, set to high to enable charge and voltage reading
#define AN_VOUT_PIN  A1 // Output voltage sensing on Vdivider
#define AN_VIN_PIN   A2 // Input voltage sensing on Vdivider
#define AN_IOUT_PIN  A3 // Output current, ADC connected to connected to TLD5542-1 IOUTMON pin (could be eventually skipped and used for NTC)
#define AN_IIN_PIN   A4 // Output current, ADC connected to connected to TLD5542-1 IINMON pin
#define AN_KEY_PIN   A0 // Keyboard voltage divider
#define AN_CAL       A5 // ADC  offset calibration input should be VREF applied to RVFBH_CAL and RVFBL_CAL voltage divider

// *************************  System defines *********************************
#define VIN_MIN        9000 // minimum input voltage
#define VIN_MAX        30000// MAX input voltage
#define VO_MIN         3500 // minimum output voltage, do not set lower. if lower value is needed then bypass reverse protection mosfet Q6 on the TLd5542-1CHG_SHILED schematic version S05
#define VOUT_OVERV     30000// MAX output voltage, during open load, with resistor divider on VFB RFBL=10k RFBH=220k  it would be 30,6V, use 30V as threshold to detect if no load is connected
#define VO_SHORT       3000 // output voltage below this value is considered a short.
#define IO_MAX (150000/RSHO)// [mA]Max output current
#define IO_MIN   (IO_MAX/20)// [mA]Min output current
#define PIN_MIN         10  // minimum input power
#define PIN_MAX         65  // MAX input power, available only at 20V input voltage
#define DUTY_MIN        2   // minimum Duty cycler
#define DUTY_MAX        100 // MAX Duty Cycle
#define IIN_MAX         5500// [mA] MAX input current , limited to 6.2 A (peak includes ripple ) in buck boost and 9.3A in booost mode by the RSWCS 8mOhm resistor on the TLD5542-1CHG_SHIELD
#define RESCAN_DELAY    10  // [ms]
#define KEY_SOUND_MS    10
#define REPEAT_TOUCH_MS 100 // [ms] start repeating key on a long touch after this period
#define INCREASE_STEP_MA 250// power derating derating output current increase steps (V target / STEP) 
#define DECREASE_STEP_MV 250// power derating derating output current decrease steps (V target / STEP) 
#define IO_STEP    IO_MAX/100// output voltage inc/dec step

#define EEPROM_INIT  0xFECA //dummy word used to check if user parameters has been written on the EEPROM at least once, in LED DRV (use different on other sketches)

// ********************  TYPE DEFINITIONS ************************************

// BUZZER FREQUENCIES ENUM
// allowed frequencies [Hz] for use with specific buzzer (PS1240P02BT) to achieve higher efficiency 
typedef enum freqEnum 
{
  FREQ_LOW      = 1500, 
  FREQ_MID_LOW  = 2500, 
  FREQ_MID      = 4000,
  FREQ_HIGH     = 5000, 
 }frequency;

//  key buttons defines ENUM
// allowed frequencies [Hz] for use with specific buzzer (PS1240P02BT) to achieve higher efficiency. 
enum keyEnum 
{
  btnNONE   =0,
  btnENTER  =1, 
  btnUP     =2, 
  btnDOWN   =3,
  btnMODE   =4, 
 };

// FSM machine states  
typedef enum FSMstateEnum// charger finite state machine states enum
{ SELF_CHECK, 
  INIT, 
  OUTPUT_ON, 
  FAULT
}FSMstate;
 
// chgErrEnum: Possible error code on the charger enum
// being in different bit position it is easy to distinguish when multiple error are happening
enum chgErrEnum
{ NO_ERROR        = 0x0000,// task, or function performed without errors
  ERR_SPI         = 0x0001,// SPI bus error        
  ERR_IN_VOLT     = 0x0002,// input voltage error     
  ERR_TLD         = 0x0004,// TLD5542-1 error
  ERR_SHORT       = 0x0008,// short circuit at the output
  ERR_BAT_OUT     = 0x0010,// battery connected at the output of a voltage regulator
  ERR_SHIELD      = 0x0020,// battery connected at the output of a voltage regulator
  ERR_OPEN        = 0x0040,// short circuit at the output
  ERR_GENERIC     = 0X8000 // generic error 
};    

//paramIndex enum: selectable user paramenters index, while navigating in the charger setup menu
enum paramIndex 
{ PARAM_IOUT,      // SET output voltage
  PARAM_PI_MAX,     // maximum charger input current 
  PARAM_DUTY,
  MAX_PARAM};

//userVarType: struct that contains all the user set variables
typedef struct {
  unsigned int IoNom; // [mV] Nominal requested output voltage (will be lowered in case max power is exceeded)
  unsigned int PiMax; //[W] max input power
  unsigned int Duty;  //[%] max input power
}userVarType;

//chrgVarType: struct that contains all the charger variables
typedef struct {
  unsigned int  IiMon;
  unsigned int  outON;// indicate if the output is on
  unsigned int  IoMon;// ADC reading from the TLD5542-1 IOUTMON pin
  unsigned int  VoMon;
  unsigned int  IoSet; // [mV] Set output voltage
  unsigned int  ViMon;
  unsigned int  PinMon;
  uint8         tldStatus;
  chgErrEnum    err;
  uint32_t      usbPDOcurr=0xDEADBEEF;
  uint16_t      USBmAMax; 
  uint16_t      USBVout; 
}chrgVarType;

// ********************  GLOBAL VARIABLE DECLARATIONS **************************
int         ADCoffs     =0; // arduino offset in mV, calculated by 
int         iInMonOffs  =0; // TLD5542-1 iinmonitor offset
int         iOutMonOffs =0 ;// TLD5542-1 iinmonitor offset
userVarType userVar{1000,   // [mA] default output current
                    50,     // [W]default input power
                    100};   // [%] default pwm %
chrgVarType chrgVar;

// USB BCR device instance CYPD3177
BBI2C   bbi2c_CYP; // I2C instance for CYPD3177 USB BCR device
#define CYP_I2C_ADDR 0x08
#define CYP_CURRENT_PDO_ADDR 0x1010
#define BITMASK10 0x03FF
#define CURRENT_PDO_ADDRH 0x10 // current PDO register address highest 8 bits
#define CURRENT_PDO_ADDRL 0x10 // current PDO register address highest 8 bits

//OLED DISPLAY instance
static uint8_t *ucBackBuffer = NULL;
OBDISP         obd;
char           sTmp[32];// temporary string for OLED writing
#define RESET_PIN -1    // let OneBitDisplay figure out the display address. Set this to -1 to disable or the GPIO pin number connected to the reset line 
#define OLED_ADDR -1    // unknown OLED address, use I2C discovery
#define FLIP180    0    // don't rotate the display
#define INVERT     0    // don't invert the display
#define USE_HW_I2C 0    // Bit-Bang the I2C bus
#define MY_OLED OLED_128x64 
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ********************  FUNCTION DECLARATIONS *******************************
uint16 readVoltageDivider(int AnalogInput, uint16 rvfbl, uint16 rvfbh);
int    read_buttons();
void   setParam(int lcd_key,int paramNum );
void   calADCoffset();
void   calIINMON_IOUTMONoffset();
uint16 readADCWithOffs(int AnalogInput);
int    readIinMon();
uint16 touchSingle();

// ********************  FUNCTION DEFINITIONs *******************************
void setup() 
{
  int tmp, rc;
  uint8_t reg_addr[2]= {CURRENT_PDO_ADDRH,CURRENT_PDO_ADDRL};// USB BCR CYPD3177 Register Address of the USB-PD CURRENT_PDO register, where it is present the max current and voltage provided on the USB-PD adater(B19..B10 voltage in 50mV unit) B9..B0 Max current in 10mA units
  uint8_t reg_data[4]= {0xDE,0xAD,0xBE,0xEF};// Dummy word to be overwritten when a register is read correctly
  int     ret_read; 
  
  onSound();  // generate wake up sound
  analogReference(EXTERNAL);// set analog reference external 
    
  // Setup for the parameters for SPI communication, set CSN and EN pins as outputs and set EN to High  
  Serial.begin(9600);                     // sets the baud rate for communication with the computer to 9600 bauds
  pinMode(CSN_TLD_PIN, OUTPUT);           // sets CSN as output
  digitalWrite(CSN_TLD_PIN, HIGH);        // prepare CSN = high o be ready for the first SPI command
  pinMode(TLD_EN_PIN, OUTPUT);            // sets EN as output
  pinMode(TLD_PWMI_PIN, OUTPUT);          // sets PWMI as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GHRG_EN_PIN, OUTPUT);           // 
  pinMode(AN_IIN_PIN, INPUT);
  pinMode(BUZZER, OUTPUT); // Set buzzer - pin as an output

  //setup OLED: If SDA and SCL pins are specified, they would be bit-banged in software, otherwise uses standard I2C bus at 400Khz
  rc = obdI2CInit(&obd, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 800000L); 
  
  if (rc != OLED_NOT_FOUND){
    obdSetBackBuffer(&obd, ucBackBuffer);
    obdFill(&obd, 0x0, 1);//clear OLED 
    obdWriteString(&obd, 0,0,0,(char *)"     LITIX   ", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,2,(char *)"   TLD5542-1 ", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,4,(char *)"   LED DRIVER", FONT_NORMAL, 0, 1); 
    obdWriteString(&obd, 0,0,6,(char *)"     V1.03  ", FONT_NORMAL, 0, 1); 
    delay(2000);  
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
  
  SPI.begin();               // Initializes the SPI bus for the TLD5542-1 (not really mandatory because OLED library it is intializing)
  
  calADCoffset();            //calculate ADC offset based on AN_CAL expected voltage
  calIINMON_IOUTMONoffset();//calculate IINMON offset 
}

// void sound: generate a beep on the buzzer, at freq frequency
// it is using the enum in order to force the user using resoinances of buzzer
void sound(frequency freq,int ms)
{
  tone(BUZZER, freq); // Send 1KHz sound signal...
  delay(ms );// call delay not in power safe 
  noTone(BUZZER);// stop sound (by stopping compare module)
}
// function onSound: generates "enabling" sound withthe buzzer. E.G. use when the device wakes up
void offSound()
{ 
  sound(FREQ_HIGH, 60);
  delay (60);// pause between each sound
  sound(FREQ_LOW, 60);  
}
// function offSound: generates "disabling" sound. E.G. use when battery is detached
void onSound()
{ 
  sound(FREQ_LOW, 60);
  delay (60);// pause between each sound
  sound(FREQ_HIGH, 60);  
}

// showScreenADC:use when testing the HW for debugging. 
// shows ADC readings, exit if a button is pressed
// NOTE: it leaves TLD5542-1 ON
void showScreenADC() 
{ 
  int lcd_key;
  uint16 anVout,anVin,anIout, anIin,anKey,anCal,anIinTMP;
  uint16 anVoutRaw,anVinRaw,anIoutRaw, anIinRaw,anKeyRaw;
  static int i = 0; // counter to show ADC sample count
  
  digitalWrite(TLD_PWMI_PIN, LOW);       // ensure PWMI =0
  digitalWrite(TLD_EN_PIN, HIGH);        // turn ON the TLD5542-1 to have IOUTMON
  while(1){
    // update adc radings input and output
    anVout = readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH); 
    anVin  = readVoltageDivider(AN_VIN_PIN,(uint32)RVIFBL,(uint32)RVIFBH); 
    anIin  = readADCWithOffs(AN_IIN_PIN);  // read IinMon in mV
    anKey  = readADCWithOffs(AN_KEY_PIN); 
    anCal  = readADCWithOffs(AN_CAL); 
    
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
}

// showScreenSetup: Draws main screen information on OLED display highlighting selected parameter. 
// increments and decrement selected parameter calling setParam()
// @param: lcd_key UP/DOWN increase decrease parameter MODE:change selected parameter
void showScreenSetup( int lcd_key) 
{ 
  char outstr[15];
  uint8 invert=0;
  static int paramNum = PARAM_IOUT ;
  
  obdWriteString(&obd, 0,0,0,(char*)"LED DRIVER" , FONT_NORMAL, 0, 1);  
  if(chrgVar.outON == 1)
    sprintf(sTmp, " (ON) " );
  else
    sprintf(sTmp, " (OFF)");
  obdWriteString(&obd, 0,-1,-1,sTmp , FONT_NORMAL, 0, 1); 

  obdWriteString(&obd, 0,0,2,(char*)" Io   PiMax Duty", FONT_NORMAL, 0, 1);  // use previous cursor position
 
  invert = (paramNum == PARAM_IOUT);
  sprintf(sTmp, "%d.%02dA",userVar.IoNom/1000,(userVar.IoNom%1000)/10);
  obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, invert, 1);  // use previous cursor position
  
  invert = (paramNum == PARAM_PI_MAX);
  sprintf(sTmp, " %dW  ",userVar.PiMax);
  obdWriteString(&obd, 0,-1,3,sTmp , FONT_NORMAL, invert, 1);  // use previous cursor position

  invert = (paramNum == PARAM_DUTY);
  sprintf(sTmp, " %d%% ",userVar.Duty);
  obdWriteString(&obd, 0,-1,3,sTmp , FONT_NORMAL, invert, 1);  // use previous cursor position

  if(chrgVar.USBmAMax !=0){ // USB-C with power delivery is applied show maximum power
    sprintf(sTmp, "USB %dmA %dV ",chrgVar.USBmAMax,chrgVar.USBVout);
    obdWriteString(&obd, 0,0,5,sTmp , FONT_NORMAL, 0, 1);  
  }

  sprintf(sTmp, "Pi%dW ",chrgVar.PinMon );
  obdWriteString(&obd, 0,0,6,sTmp , FONT_NORMAL, 0, 1); 
  sprintf(sTmp, "Ii%dmA ", chrgVar.IiMon );
  obdWriteString(&obd, 0,10*6,6,sTmp , FONT_NORMAL, 0, 1); 
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

// setParam: increments and decrement selected parameter calling incDec()
// @param: lcd_key UP/DOWN increase decrease parameter 
void setParam(int lcd_key, int paramNum )
{
  switch(paramNum){        
  case PARAM_IOUT:// mV
    userVar.IoNom = incDec(userVar.IoNom, IO_MAX, IO_MIN ,IO_STEP,  lcd_key );
    break;
  case PARAM_PI_MAX:
    userVar.PiMax = incDec(userVar.PiMax, PIN_MAX, PIN_MIN,5, lcd_key );
    break;
  case PARAM_DUTY:
    userVar.Duty = incDec(userVar.Duty, DUTY_MAX, DUTY_MIN,2, lcd_key );
    break;
  }    
}

// incDec: increment or decrement input value wrapping around on Max val
// @param: input, input number to be incremented/decremented
// @param: limit_max, upper limit for incrementing
// @param: limit_min, lower limit for decrementing
// @param: increment, increment step
// @param: key, keyboard input:  btnUP increase , btnDWN decrease
unsigned int incDec(uint16 input, uint16 limit_max,uint16 limit_min, uint16 increment, uint16 key)
{
  if (key==btnUP )
    if (input<=limit_max-increment)
      input+=increment;
    else
     input=limit_max;
  if (key==btnDOWN )
    if(input>=limit_min+increment )
      input-=increment;
    else
      input=limit_min;
  return input;
}  
   
// printError: prints the error string message, on the OLED, in the given x,y position
void printError(int x, int y)
{
  switch(chrgVar.err){  
  case NO_ERROR:
    sprintf(sTmp, "NO ERROR      ");
    break;      
  case ERR_SPI:
    sprintf(sTmp, "ERR SPI       ");
    break;
  case ERR_IN_VOLT:
    sprintf(sTmp, "ERR IN VOLT   ");
    break;
  case ERR_TLD:
    sprintf(sTmp, "TLD5542-1 ERR ");
    break;
  case ERR_SHORT:
    sprintf(sTmp, "OUTPUT SHORT ");
    break;
  case ERR_OPEN:
    sprintf(sTmp, "OUTPUT OPEN ");
    break;
  case ERR_BAT_OUT:
    sprintf(sTmp, "HIGH OUT VOLT ");
    break;
  case ERR_SHIELD:
    sprintf(sTmp, "ERR SHIELD VREG ");
    break;
  case ERR_GENERIC:
    sprintf(sTmp, "GENERIC ERROR");
    break;
  default:
    sprintf(sTmp, "unknown ERR");
    break;  
  }    
  obdWriteString(&obd, 0,x,y,sTmp , FONT_NORMAL, 0, 1); 
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
   obdWriteString(&obd, 0,0,1,sTmp , FONT_NORMAL, 0, 1);  
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
  static FSMstateEnum state = SELF_CHECK, prevState;// finite state machine current state and previous state
  int    lcd_key     = btnNONE;
  uint32 PinMon;
  static uint16 tmp;

   // update adc radings input and output
  chrgVar.VoMon = readVoltageDivider(AN_VOUT_PIN,(uint32)RVOFBL, (uint32)RVOFBH); 
  chrgVar.ViMon = readVoltageDivider(AN_VIN_PIN,(uint32)RVIFBL,(uint32)RVIFBH); 
  if (digitalRead(TLD_EN_PIN)==1){
    chrgVar.IiMon = readIinMon(); // read iinmon only when TLD5542-1 is enabled
    chrgVar.PinMon= ((uint32)chrgVar.IiMon * (uint32)chrgVar.ViMon)/1000000;
  }
  else{ // if TLD5542-1 is off, I In monitor is not working, so set to 0
    chrgVar.IiMon = 0;
    chrgVar.PinMon= 0;
  }

  // check if input voltage is OK
  if (chrgVar.ViMon < VIN_MIN || chrgVar.ViMon > VIN_MAX ){// if VIN it is not on the allowed range, sigal it and go to FAULT state 
    chrgVar.err = ERR_IN_VOLT;
    state=FAULT;
  }
  //showScreenADC();      // debug use only to check ADC accuracy during debug
  lcd_key = touchSingle();// read keypad

  if (state != prevState)    // every time FSM machine change state
       obdFill(&obd, 0x0, 1);// clear display
  prevState = state;         // update FSM state history, in order to detect if state has been changed

  switch(state){
    case SELF_CHECK: // check if HW is working 
      digitalWrite(TLD_PWMI_PIN, LOW);  // turn off PWMI 
      digitalWrite(TLD_EN_PIN, HIGH);   // turn ON the TLD5542-1
      delay(5);
      TLD5542_1_Sync_Read_Register(TLD5542_1_STANDARD_DIAGNOSIS_ADDR);
      chrgVar.tldStatus = tld5542_1.STANDARD_DIAGNOSIS.STATE;
      obdFill(&obd, 0x0, 1);//clear OLED
      if(chrgVar.tldStatus!=TLD5542_1_ACTIVE ){
        chrgVar.err = ERR_TLD;
        state=FAULT;
      }
      else{
        if(chrgVar.VoMon<VO_MIN){        // if there are no voltage at the output startup , to avoid batteries at the output of the LED driver
          if ( verifyShieldJmpCurr() ){  // verify if the shiled is configured as current regulator
            digitalWrite(GHRG_EN_PIN, HIGH); // Enable output mosfet 
            state = INIT;
            eeprom_read_block((void*)&tmp, (void*)0, sizeof(tmp));//read user setting intitialization keyword to see if user parameter were already stored
            if (tmp == EEPROM_INIT)// eeprom with user parameter was written at least once
              eeprom_read_block((void*)&userVar, (void*)4, sizeof(userVar));
          }
          else{
            chrgVar.err = ERR_SHIELD;
            state=FAULT;
          }
        }else{        
            chrgVar.err = ERR_BAT_OUT;
            state=FAULT;
          }
      }
      break;
       
    case INIT: // shows screen setup, requesting user parameters
      // check charger OK
      chrgVar.outON=0;// set output to 0
      digitalWrite(TLD_EN_PIN, LOW);       // turn off the TLD5542-1
      digitalWrite(TLD_PWMI_PIN, LOW);     // set PWMI to low to avoid Vout increase
      showScreenSetup(lcd_key);// request for parameters, keep output off
      if ( lcd_key == btnENTER){ // if enter button turn on VOUT
        // store user setting in the eprom every time the output is enabled
        tmp = EEPROM_INIT;//set tmp to a know keyword to check if EEPROM is correctly read
        eeprom_write_block((const void*)&tmp, (void*)0, 4 );//save the know key to show EEPROM ihas been written at least once
        eeprom_write_block((const void*)&userVar, (void*)4, sizeof(userVar));//save parameter to EEPROM for next turn on
        
        digitalWrite(TLD_EN_PIN, HIGH);   // Enable TLD5542-1 device
        delay (5);
        chrgVar.IoSet = userVar.IoNom;
        TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet ,RSHO); 
        chrgVar.outON=1;// turn on the output mosfet
        analogWrite(TLD_PWMI_PIN, (userVar.Duty*255)/100);     // set PWMI high to turn on output
        state = OUTPUT_ON;
        delay (50);// wait 50ms for Vout to rise , especially needed for low analog dimming(E.G.5%), so vout check on next status is not showing a short circuit
        TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet ,RSHO);   // dummy ADIM, to clear STD diag, very likely at low ADIM a short + soft start cycle has been produced
      }
      break;

    case OUTPUT_ON:// turn on the ouput (if there are no battery connecte at the output)
      showScreenSetup(lcd_key);
      // check if output voltage is OK
      if (chrgVar.VoMon <   VO_SHORT ){ // if VIN it is below the short to GND threshold, go to FAULT state 
        digitalWrite(TLD_EN_PIN, LOW);  // turn off the TLD5542-1
        chrgVar.err = ERR_SHORT;
        Serial.print(chrgVar.VoMon );
        Serial.println("mV out  short ");
        state=FAULT;
      }
      else{
        if ( powerGood()==false ) // if maximum input power has been exceeded
            chrgVar.IoSet = chrgVar.IoSet - chrgVar.IoSet/2;// decrease output voltage according 
        else{ // if the code is here: output voltage, input current and input power are in the desired range.
              // so adjust IoSet to IoNom, increasing with 1/4 of current value steps, so in case of power derating, ramp up curve is less steep (decrease with 1/2 , increase with 1/4)
          if (chrgVar.IoSet < userVar.IoNom - chrgVar.IoSet/4) // keep IoSet below Io Nominal the one set by the user without power derating)
            chrgVar.IoSet = chrgVar.IoSet+chrgVar.IoSet/4; 
          else
            chrgVar.IoSet = userVar.IoNom;// limit at max Vout 
        }
      }
      analogWrite(TLD_PWMI_PIN, (userVar.Duty*255)/100);     // set PWMI o the desired duty
      
      //TLD5542_1_Sync_Read_Register(TLD5542_1_STANDARD_DIAGNOSIS_ADDR);// check TLD5542-1 diagnostic to see open load or short      
      TLD5542_1_analogDimmingCurrentRegulator(chrgVar.IoSet ,RSHO);   // set desired ADIM, and automatically update tld5542_1 STD diagnosis register
       
      if( tld5542_1.STANDARD_DIAGNOSIS.OUTOV ==1 ){ // check if output overvotalge flag is triggered (in case of open load
        chrgVar.err = ERR_OPEN;
        state=FAULT;
      }
      if (lcd_key == btnENTER) // interrupt if a button is presseded
        state = INIT;       
      break;
     
      case FAULT:
        digitalWrite(TLD_EN_PIN, LOW);    // turn off the TLD5542-1
        digitalWrite(GHRG_EN_PIN, LOW);   //disable battery charge
        obdWriteString(&obd, 0,0,0,(char*)"FAULT" , FONT_NORMAL, 0, 1); 
        printError(0,2);// print text error on OLED
        if( lcd_key != btnNONE)  //wait for a button to be pressed to exit the fault state
          state = SELF_CHECK;
        break;
      
    default:
      break;
  }
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
  readOffsCal = readRaw(AN_CAL);// read pure ADC value on A5
  // calcualte voltage at the pin should be (reference scaled by res divider)
  offsRef = ((uint32)REFVOLT * (uint32)RVFBL_CAL)/((uint32)RVFBH_CAL+(uint32)RVFBL_CAL); // [mV]
  //offsRef =0;//debug restore previous line
  ADCoffs =  readOffsCal - offsRef;
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
  
 // DEBUG: print calibration values 
 /* sprintf(sTmp, "iimonraw=%d  ",tmp);
  obdWriteString(&obd, 0,0,0,sTmp , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "iInMonMA= %dmA  ",readOffs);
  obdWriteString(&obd, 0,0,1,sTmp , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "iInMonOffs = %dmV  ",iInMonOffs);
  obdWriteString(&obd, 0,0,2,sTmp , FONT_NORMAL, 0, 1);  
  delay (4000);
  */
  tmp = readADCWithOffs(AN_IOUT_PIN);// read  ADC value on IOUTMO, with ADC offset compensated 
  iOutMonOffs = tmp - IOUTMON_ZERO;
/* DEBUG: print calibration values 
  sprintf(sTmp, "iOmonraw=%d  ",tmp);
  obdWriteString(&obd, 0,0,3,sTmp , FONT_NORMAL, 0, 1);  
  sprintf(sTmp, "iOMonOffs = %dmV  ",iOutMonOffs);
  obdWriteString(&obd, 0,0,4,sTmp , FONT_NORMAL, 0, 1);  
  delay(200); // wait to show acquired offsets on display
*/
  digitalWrite(TLD_EN_PIN, LOW);        // turn OFF the TLD5542-1
}

// read_buttons: Read the keypad buttons on AN_KEY_PIN resistor divider, with debounce
// buttons are supposed to be pressed individually (not more than one simultaneously)
// Buttons are producing 5/6 steps of 5V :0,83V 1,66V 2.5V 3.33V 
// @returns: return the pressed button currespondent ENUM
int read_buttons()
{
  uint32 adc_key_mV,tmp;
  int key;

  adc_key_mV = readADCWithOffs(AN_KEY_PIN);      // read the value from the sensor
  // debounce loop, prevents reading on changing keypad 
  while(1){
    delay(1);
    tmp = readADCWithOffs(AN_KEY_PIN);      // store keypad ADC in temporary variable, 
    if ( (adc_key_mV >= (tmp - tmp/16 ) ) && (adc_key_mV <= (tmp + tmp/16 ) ) ) //if the value is not changed (by 1/16) from the last read, means that it is constant
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
      delay(2);
      stillTouch=0;
    } 
    
    while(1){ // cycle will stop when all keys are released, or after REPEAT_TOUCH_MS to repeat a key 
      stillTouch++;
      delay(RESCAN_DELAY);// count rescan delay before rescanning
      if (stillTouch > (REPEAT_TOUCH_MS/RESCAN_DELAY)){
        memory=scan;
        delay (2);// rescan delay
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

// readADCWithOffs: read raw ADC value 
// no voltage divider moltiplication applied, only oversampling
// @param:AnalogInput, which ADC input is read
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
  voltage = ( ((sum * REFVOLT)>>4)/1024 ); // shift by 4 if NUM_SAMPLES is 16

  return voltage;
}
