// =============================================================================================================================
/* 
This Arduino code works on a ATMEGA328 ( Nano Uno), ATMEGA1284 chip and Nano Every
This source contains code for the following modules:  
- RTC DS3231 clock module
- LDR light sensor 5528
- HC-12 Long Range Wireless Communication Module
- DCF77 module DCF-2
The DCF77 module reads the time and date from the German longwave DCF-77 time signal
The time and date are transmitted as ASCII string as hhmmss. (031500 = 15 minutes past 3)
The date is hourly transmitted with a prefic D followed by ddmmyyyy ( D01042020 - 1 april 2020)
Comments are transmitted with a prefix @ (@ 10:07:38 09-01-2020 DCF:16)

Ed Nieuwenhuys 2020
 V06 sep 2022 Removed warnings
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
#define DCFMOD           // Use the Arduino DCF77 library with interrupts
#define DCFTINY          // Use the Tiny DCF algorithm in this program
#define HC12MOD          // Use HC12 time transreceiver Long Range Wireless Communication Module
#define LCDMOD           // LCD 2 x 16 installed 
#define MOD_DS3231       // DS3231 RTC module installed
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------

#include <Wire.h>
#include "RTClib.h"
#include "Time.h"
#include <TimeLib.h>             // For time management 
#include <EEPROM.h>
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                     # endif // LCDMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     # endif // DCFMOD
                     #ifdef HC12MOD
#include <SoftwareSerial.h>              // For HC12
                     # endif // HC12MOD  
//--------------------------------------------
// PIN Assigments
//--------------------------------------------                      
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----
 RX           = 0,                // Connects to TX
 TX           = 1,                // Connects to RX
 DCF_PIN      = 2,                // DCFPulse on interrupt pin
 PIN03        = 3,                // PIN 3
 PIN04        = 4,                // PIN 4
 PIN05        = 5,                // PIN 5
// BT_RX        = 6,                // Tx-pin on BT-mod
// BT_TX        = 7,                // Rx-pin on BT-mod 
 HC_12TX      = 6,                // HC-12 TX Pin
 HC_12RX      = 7,                // HC-12 RX Pin  
 PIN08        = 8,                // PIN 8 
 secondsPin   = 8,                // Seconds
 LED09        = 9,                // PIN 9 
 DCFgood      = 10,               // DCF-signal > 50
 PIN11        = 11,               // PIN 11
 PIN12        = 12,               // PIN 12 
 DCF_LED_Pin  = 13,               // Show DCF-signal
 };
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 PhotoCellPin = 2,                // A2
 EmptyA3      = 3,                // A3
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5};               // SCL pin

//--------------------------------------------
// HC-12 Long Range Wireless Communication Module
//--------------------------------------------
                     #ifdef HC12MOD
SoftwareSerial HC12(HC_12TX, HC_12RX); //rxPin, txPin, inverse_logic.
                                       //rxPin: the pin on which to receive serial data 
                                       //txPin: the pin on which to transmit serial data
                     # endif // HC12MOD 
//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 50
static  uint32_t msTick;          // the number of millisecond ticks since we last incremented the second counter
int     count; 
int     Delaytime = 200;
byte    Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte    Dsecond, Dminute, Dhour, Dday, Dmonth, Dyear, Dwday; 
byte    lastminute = 0, lasthour = 0, sayhour = 0;
byte    SecPulse           = 0;   // give a pulse to the Isecond led

const byte MenuItems = 7;          // No of entries in menu
const char menu[MenuItems][MAXTEXT] PROGMEM = {
 "DCF HC12 Transmitter V06",
 "Enter time as:hhmm (1321) or hhmmss (132145)",
 "A DCF Debug info On/Off", 
 "D D15122017 for date 15 December 2017",
 "G for DCF-signalinfo on display",
 "I for this info",
 "Ed Nieuwenhuys" };

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12

        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;    //RTC_DS1307 RTC; 
        #else
RTC_Millis RTCklok;   
        # endif //
DateTime Inow;
                    #ifdef DCFMOD 
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
                    #ifdef ARDUINO_AVR_NANO_EVERY
#define DCF_INTERRUPT 2               // DCF Interrupt number associated with DCF_PIN ( 2 Nano Every)
                    #else
#define DCF_INTERRUPT 0              // Nano Uno etc 
                    # endif //      
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW); // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
                    # endif // DCFMOD 

byte     DCF_signal         = 1;     // is a proper time received? (0 - 100)
                    #ifdef DCFTINY
static   uint32_t DCFmsTick;         // the number of millisecond ticks since we last incremented the second counter
uint32_t SumSecondSignal    = 0;     // sum of digital signal ( 0 or 1)
uint32_t SumSignalCounts    = 0;     // Noof of counted signals
uint32_t DiscardedSignals   = 0;     // Noof of discarded counted signals
uint32_t TimePrevious       = 0;     // Previous minute time and date
uint32_t HourMinPrevious    = 0;     // Previous minute time

byte     Bitpos             = 0;
byte     StartOfMinute      = 0;
byte     StartOfEncodedTime = 0;
byte     Receivebit         = 0;
byte     Paritybit          = 0;
int      LastGateValid      = 0;      // Counter to start recording a DCF-signal after DCFValidGate received
int      DCFValidGate       = 100;    // If LastGateValid reaches this value a DCF-signal is recorded
byte  MinOK, HourOK, YearOK = 0;
                    # endif // DCFTINY
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
                    # endif // 
//----------------------------------------
// Common
//----------------------------------------
byte PrintDebugInfo         = 0;      // for showing debug info for DCFTINY
char sptext[MAXTEXT+2];               // for common print use    
int  MilliSecondValue       = 999;    // The duration of a second  minus 1 ms. Used in Demo mode
uint32_t Ed =0, Ellenbaas = 0;
// --------------------------------------------------------------------------
// End Definitions                                                    
// --------------------------------------------------------------------------
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
 SerialCheck(); 
 EverySecondCheck();
 EveryMinuteUpdate();
//                              #ifdef DCFMOD         
 DCF77Check();
//                              # endif // DCFMOD
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                        
 Serial.begin(9600);                                                // Setup the serial port to 9600 baud                                                        // Start the RTC-module  
 Wire.begin();                                                      // Start communication with I2C / TWI devices                            
 pinMode(DCF_LED_Pin, OUTPUT);                                      // For showing DCF-pulse or other purposes
 pinMode(DCF_PIN,     INPUT_PULLUP);
// pinMode(BT_State,    INPUT_PULLUP);
// pinMode(BT_Break,    OUTPUT );   
 pinMode(DCFgood,     OUTPUT );  
 pinMode(secondsPin,  OUTPUT );
                              #ifdef MOD_DS3231
 RTCklok.begin();                                                   // start the RTC-module
                              # else
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                 // if no RTC module is installed use the ATMEGAchip clock
                              # endif // MOD_DS3231  
                              #ifdef DCFMOD
 DCF.Start();                                                      // Start the DCF-module
 Tekstprintln("DCF enabled");
                              # endif // DCFMOD
                     #ifdef DCFTINY
 DCFmsTick = millis();                                              // Start of DCF 1 second loop
                     # endif // DCFTINY
                              #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                         // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                              # endif // LCDMOD
      #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("ARDUINO_AVR_NANO_EVERY enabled");
       # endif //
      
                              #ifdef HC12MOD
 HC12.begin(9600);               // Serial port to HC12
 Tekstprintln("HC-12 time sender enabled");
                             # endif // HC12MOD 
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
// SumSecondSignal = SumSignalCounts = 0;
 msTick = millis();                                      // Used in KY-040 rotary for debouncing and seconds check
 SWversion();                                                       // Display the version number of the software
 GetTijd(0);                                                        // Get the time and store it in the proper variables
//  Setup_BluetoothCentralMode();
 } 
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
 for (int i = 0; i < MenuItems; i++)   {strcpy_P(sptext, menu[i]);   Tekstprintln(sptext);  }
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char const *tekst)
{
  Serial.print(tekst);     
}
void Tekstprintln(char const *tekst)
{
 Serial.println(tekst);  
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
 if ( millis() - msTick > 50)   { digitalWrite(secondsPin,LOW); }  // Turn OFF the second on pin 13
 if ( millis() - msTick > 999)                                     // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                              // second++; 
   digitalWrite(secondsPin,HIGH);                                  // turn ON the second on pin 13
   ++SecPulse;                                                     // second routine in function DimLeds
   GetTijd(0);                                                     // synchronize time with RTC clock      
                           #ifdef LCDMOD
   Print_tijd_LCD();
                           # endif // LCDMOD  
                           #ifdef HC12MOD  // disabled with the x HC12MODx
   if( (Inow.second()==10) )               // When HC12 sends it disturbs DCF-signal. In seconds 1 - 19 are whether info bits
    {                                      // Now we can send some info
     sprintf(sptext,"@ %0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d DCF:%0.2d",
            Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year(),DCF_signal);
     HC12.print(sptext);     
    }    
                            # endif // HC12MOD
  }
}
 
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                    // Show time every minute
  { 
   lastminute = Iminute;
   Print_RTC_tijd();  
   DCF_signal--;
   DCF_signal = constrain( DCF_signal,1,99);
//        sprintf(sptext,"Ed : Thijs %ld : %ld",Ed , Ellenbaas );    Serial.println(sptext); 
        //HC12.println(sptext);
        PrinttijdTo_HC12(); 
  } 
 if (Ihour != lasthour) {lasthour = Ihour;}
 }
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 byte LHbit = 1 - digitalRead(DCF_PIN);        // write inverted DCF pulse to LED on board 
 digitalWrite(DCF_LED_Pin, LHbit ); 
                             #ifdef DCFMOD
 time_t DCFtime = DCF.getTime();               // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   Tekstprintln("DCF: Time is updated ----->  ");
   sprintf(sptext,"%0.2d%0.2d%0.2d",Inow.hour(),Inow.minute(),Inow.second());
   Tekstprintln(sptext); 
   DCF_signal+=2;
   setTime(DCFtime); 
   RTCklok.adjust(DCFtime);
//   Ellenbaas++;
  }
                             # endif // DCFMOD                             
                             #ifdef DCFTINY
 SumSignalCounts += 1;                         // Noof of counted signals  
 if(LHbit) 
   {
    LastGateValid++; 
    if (LastGateValid > DCFValidGate ? SumSecondSignal++ : DiscardedSignals++ );
   }
 if ( millis() - DCFmsTick > 999)              // Compute every second the received DCF-bit to a time 
    { 
      DCFmsTick = millis(); 
      if(byte OKstatus = UpdateDCFclock())     // if after 60 sec a valid time is calculated, sent it to the HC-12 module
         {
          if(OKstatus == 2)                    // if time flag was OK and date flag was NOK
            {
             sprintf(sptext," TIME OK --> %0.2d:%0.2d",Dhour, Dminute );
             Serial.println(sptext);
             RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Dhour, Dminute, 0));
            }
           else                                // if time flag was OK and date flag also OK  (OKstatus == 1)
            {
            sprintf(sptext," TIME OK --> %0.2d:%0.2d %0.2d-%0.2d-%0.4d/%0.1d",Dhour, Dminute, Dday, Dmonth, 2000+Dyear, Dwday );
            Serial.println(sptext);
            RTCklok.adjust(DateTime(2000+Dyear, Dmonth, Dday, Dhour, Dminute, 0));
            }         
          DCF_signal+=2;
 //         Ed++;
         }
    } 
                             # endif // DCFTINY                  
 DCF_signal = constrain(DCF_signal,0 ,100);    // DCF_signal <100
 analogWrite(DCFgood, DCF_signal); 
} 
                             #ifdef DCFTINY
//--------------------------------------------
// CLOCK Make the time from the received DCF-bits
//--------------------------------------------
byte UpdateDCFclock(void)
{
 byte TimeOK;              // return flag is proper time is computed
 if (Bitpos >= 60) {Bitpos=0; StartOfMinute = 0;} 
 if (PrintDebugInfo)
    { 
    sprintf(sptext,"@ %5ld %5ld %3ld %2ld%% Bitpos: %2d",
    SumSecondSignal, SumSignalCounts, DiscardedSignals,  100 * SumSecondSignal/ SumSignalCounts, Bitpos );
    Serial.print(sptext);
    }  
 if (SumSignalCounts>5000 && SumSecondSignal<500) Bitpos = 59;    // if enough signals the one second no signal is found. this is position zero
 Receivebit = 9;
 int msec = (int)1000 * SumSecondSignal/ SumSignalCounts;   
 switch(msec)
  {
   case   0 ...  50:  Receivebit = 9; break;  // if signal is less than 0.05 sec long than it is nothing
   case  51 ... 150:  Receivebit = 0; break;  // if signal is 0.1 sec long than it is a 0 
   case 151 ... 250:  Receivebit = 1; break;  // if signal is 0.1 sec long than it is a 1 
   default:           Receivebit = 2; break;  // in all other cases it is an error
  }
 if (PrintDebugInfo){ sprintf(sptext," Bit: %1d ",Receivebit );   Serial.print(sptext);}
 switch (Bitpos)                              // Ddecode the 59 bits to a time and date.
  {                                           // It is not binary but "Binary-coded decimal" 
                                              // and blocks are checked with an even parity bit. 
   case   0: if(Receivebit==0)       StartOfMinute = 1;                    break; // Bit must always be 0
   case   1 ... 19: Paritybit = 0;                                         break;
   case  20: if(Receivebit==1)
               {
                StartOfEncodedTime = 1;                      // Bit must always be 1
                Dsecond = Dminute = Dhour = Dday =Dwday = Dmonth = Dyear = 0;
                }
              else StartOfEncodedTime = 0;
                                                                           break;
   case  21: if(Receivebit==1) {Dminute  = 1;  Paritybit = 1 - Paritybit;} break;
   case  22: if(Receivebit==1) {Dminute += 2 ; Paritybit = 1 - Paritybit;} break;
   case  23: if(Receivebit==1) {Dminute += 4 ; Paritybit = 1 - Paritybit;} break;
   case  24: if(Receivebit==1) {Dminute += 8 ; Paritybit = 1 - Paritybit;} break;
   case  25: if(Receivebit==1) {Dminute += 10; Paritybit = 1 - Paritybit;} break;
   case  26: if(Receivebit==1) {Dminute += 20; Paritybit = 1 - Paritybit;} break;
   case  27: if(Receivebit==1) {Dminute += 40; Paritybit = 1 - Paritybit;} break;
   case  28: if(Receivebit==Paritybit) 
               {
                 if (PrintDebugInfo)
                  {                
                   sprintf(sptext,"(Min Parity OK %d %d)" ,Receivebit, Paritybit);
                   Serial.print(sptext);
                  }
                MinOK = 1;
               }
                else 
               {
                 if (PrintDebugInfo)
                  {                
                   sprintf(sptext,"(Min Parity NOK %d %d)",Receivebit, Paritybit);
                   Serial.print(sptext);
                  }
                MinOK = 0;
               }           
             Paritybit = 0;                                               break;  
   case  29: if(Receivebit==1) {Dhour   =  1; Paritybit = 1 - Paritybit;} break;
   case  30: if(Receivebit==1) {Dhour  +=  2; Paritybit = 1 - Paritybit;} break;
   case  31: if(Receivebit==1) {Dhour  +=  4; Paritybit = 1 - Paritybit;} break;
   case  32: if(Receivebit==1) {Dhour  +=  8; Paritybit = 1 - Paritybit;} break;
   case  33: if(Receivebit==1) {Dhour  += 10; Paritybit = 1 - Paritybit;} break;
   case  34: if(Receivebit==1) {Dhour  += 20; Paritybit = 1 - Paritybit;} break;
   case  35: if(Receivebit==Paritybit) 
                {
                 if (PrintDebugInfo)
                   {
                    sprintf(sptext,"(Hour Parity OK %d %d)", Receivebit, Paritybit);
                    Serial.print(sptext);
                   }                       
                  HourOK = 1;
                }
               else 
                {
                  if (PrintDebugInfo)
                   {
                    sprintf(sptext,"(Hour Parity NOK %d %d)",Receivebit, Paritybit); 
                    Serial.print(sptext);
                   }                  
                  HourOK = 0;
                }               
             Paritybit = 0;                                              break;  
   case  36: if(Receivebit==1) {Dday    =  1; Paritybit = 1 - Paritybit;} break;
   case  37: if(Receivebit==1) {Dday   +=  2; Paritybit = 1 - Paritybit;} break;
   case  38: if(Receivebit==1) {Dday   +=  4; Paritybit = 1 - Paritybit;} break;
   case  39: if(Receivebit==1) {Dday   +=  8; Paritybit = 1 - Paritybit;} break;
   case  40: if(Receivebit==1) {Dday   += 10; Paritybit = 1 - Paritybit;} break;
   case  41: if(Receivebit==1) {Dday   += 20; Paritybit = 1 - Paritybit;} break;
   case  42: if(Receivebit==1) {Dwday   =  1; Paritybit = 1 - Paritybit;} break;
   case  43: if(Receivebit==1) {Dwday  +=  2; Paritybit = 1 - Paritybit;} break;
   case  44: if(Receivebit==1) {Dwday  +=  4; Paritybit = 1 - Paritybit;} break;
   case  45: if(Receivebit==1) {Dmonth  =  1; Paritybit = 1 - Paritybit;} break;
   case  46: if(Receivebit==1) {Dmonth +=  2; Paritybit = 1 - Paritybit;} break;
   case  47: if(Receivebit==1) {Dmonth +=  4; Paritybit = 1 - Paritybit;} break;
   case  48: if(Receivebit==1) {Dmonth +=  8; Paritybit = 1 - Paritybit;} break;
   case  49: if(Receivebit==1) {Dmonth += 10; Paritybit = 1 - Paritybit;} break;
   case  50: if(Receivebit==1) {Dyear   =  1; Paritybit = 1 - Paritybit;} break;
   case  51: if(Receivebit==1) {Dyear  +=  2; Paritybit = 1 - Paritybit;} break;
   case  52: if(Receivebit==1) {Dyear  +=  4; Paritybit = 1 - Paritybit;} break;
   case  53: if(Receivebit==1) {Dyear  +=  8; Paritybit = 1 - Paritybit;} break;
   case  54: if(Receivebit==1) {Dyear  += 10; Paritybit = 1 - Paritybit;} break;
   case  55: if(Receivebit==1) {Dyear  += 20; Paritybit = 1 - Paritybit;} break;
   case  56: if(Receivebit==1) {Dyear  += 40; Paritybit = 1 - Paritybit;} break;
   case  57: if(Receivebit==1) {Dyear  += 80; Paritybit = 1 - Paritybit;} break;
   case  58: if(Receivebit==Paritybit) 
               {
                 if (PrintDebugInfo)
                   { 
                    sprintf(sptext,"(Year Parity OK %d %d0", Receivebit, Paritybit );
                    Serial.print(sptext);
                   }
                 YearOK = 1;
               }
             else
               {
                if (PrintDebugInfo)
                  {
                   sprintf(sptext,"(Year Parity NOK %d %d)",Receivebit, Paritybit ); 
                   Serial.print(sptext);
                  }
                 YearOK = 0;
               }
              Paritybit = 0;                                              break;  
    case  59: //Serial.print("silence");
    default:                                                              break;
   }
 Bitpos++;
 if (PrintDebugInfo)
    {
     sprintf(sptext," %0.2d:%0.2d %0.2d-%0.2d-%0.4d/%0.1d %d",Dhour, Dminute, Dday, Dmonth, 2000+Dyear, Dwday, Paritybit);
     Serial.println(sptext);
    }
 SumSecondSignal = SumSignalCounts = DiscardedSignals = LastGateValid = DiscardedSignals = 0;
 if(Bitpos == 59)
   {
   if(StartOfEncodedTime && MinOK && HourOK && YearOK)
     {                                          // check is the time differ only one minute from the previous time
      uint32_t TimeNow = (((((Dyear * 12 + Dmonth) * 31) + Dday) * 24) + Dhour) * 60 + Dminute;
//      sprintf(sptext," Time now  %ld  :: Previous  %ld",TimeNow , TimePrevious); Serial.println(sptext);
      if(TimeNow - TimePrevious <= 2)           // Time is valid if difference with previous time is one minute
        {
         TimeOK = 1;    
        }
      TimePrevious = TimeNow;      
      } 
    else                                        // Time is not valid
      {
       if(StartOfEncodedTime && MinOK && HourOK)
         {                                      // check is the time differ only one minute from the previous time
          uint32_t TimeNow = Dhour * 60 + Dminute; 
                                                // sprintf(sptext," HourMinTime now  %ld  :: Previous  %ld",TimeNow , HourMinPrevious); Serial.println(sptext);
          if(abs(TimeNow - HourMinPrevious <= 2))    // Time is valid if difference with previous time is one minute
            {
             TimeOK = 2;    
            }
          HourMinPrevious = TimeNow;         
          }
        else                                    // if no vslid time stamp add one minute to time previous
          {
           TimePrevious++;
           TimeOK = 0;       
          }
       } 
   }                                            // --- end if(Bitpos == 59)
return(TimeOK);
}                 
                             # endif // DCFTINY
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String  SerialString; 
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;       // allow input from Space - Del
  }
 if (SerialString.length()>0) 
         ReworkInputString(SerialString);      // Rework ReworkInputString();
 SerialString = "";
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTCklok.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d DCF:%0.2d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year(),DCF_signal);
 Tekstprintln(sptext);
}
                    #ifdef LCDMOD
//--------------------------------------------
// CLOCK Print time to LCD display
//--------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Inow.hour(),Inow.minute(),Inow.second());   lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                          lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%0.2d-%0.2d-%0.4d",Inow.day(),Inow.month(),Inow.year());       lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                        lcd.print(sptext);
}
                    # endif // LCDMOD
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}

//--------------------------------------------
// HC-12 utility function prints time to HC-12
//--------------------------------------------
void PrinttijdTo_HC12(void)
{
 static uint32_t HC12timerloop;                       // Serial.println(HC12timerloop);
                              #ifdef HC12MOD
 while ( ( millis() - HC12timerloop) < 100) delay(1); // Wait 100 msec before entering this function again
 HC12timerloop = millis();
 sprintf(sptext,"%0.2d%0.2d%0.2d\r\n",Inow.hour(),Inow.minute(),Inow.second());
 HC12.println(sptext);                                // Serial.println(sptext);
                            # endif // HC12MOD
}
//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  , 0,24);
 Iminute = constrain(Iminute, 0,59); 
 Isecond = constrain(Isecond, 0,59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                                      // synchronize time with RTC clock
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
 Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
 Wire.write(0x11);
 Wire.endTransmission();
 Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
 if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & 0b01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
 else {  temp3231 = -273; }   
 return (temp3231);
}

// ------------------- End  Time functions 


//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64 )                                           // Does the string start with a letter?
  {
  int val = InputString[0];
  int FMfreq;
  
  Tekstprintln(sptext);
  switch (val)
   {
    case 'A':
    case 'a':
             PrintDebugInfo = 1 - PrintDebugInfo;
             break;  
    case 'D':
    case 'd':  
             if (InputString.length() == 9 )
              {
               int Jaar;
               temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
               temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
               temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
               Iday   = constrain(Iday  , 0, 31);
               Imonth = constrain(Imonth, 0, 12); 
               Jaar   = constrain(Jaar , 1000, 9999); 
               RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
               sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
               Tekstprintln(sptext);
              }
              else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");
             break;         
    case 'I':
    case 'i':   
             SWversion();
             break;      
    default:
             break;
    }
  }
   else if (InputString.length() > 3 && InputString.length() <7 )
      {
       temp = InputString.substring(0,2);   
       Ihour = temp.toInt(); 
       if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = temp.toInt(); }
       if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = temp.toInt(); }
       SetRTCTime();
      }
 InputString = "";
 temp = "";
}
