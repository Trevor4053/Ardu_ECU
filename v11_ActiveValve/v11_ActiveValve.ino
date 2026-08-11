/*
//Rev3 - Added Trial modes trialModeStarter and trialModeFuelPump to determine the best settings for Final Setup
//Rev4 - use pulse counter module intead or interrupt to measure turbine RPM
//Rev5 - pulse counter code updated
//Rev6 - Webserver based data terminal functionality added
//Rev7 - Cleaned up the code and added 2 more data screens in webserver (Page3&4)
//Rev8 - changed output control levels to (outMin-outMax) instead of (0-100) previously.Added data screen 5 for abort parameters
//Rev9 - Added Temperature tolerance for Trial mode Fuel Pump Control- Added support for brushless motor by introducing starterLimit
          when "starterLimit" > 0,  "starterLimit" will be used to determine when the starter motor stops and when gas valve is closed
          when "starterLimit"==0, starter motor and gas valve cutoff will be according to defined RPM in the settings
          This is done because we found that magnetic RPM sensing is not correct due to brushless motor EM noise.
          The issue is not present when a DC brushed motor is used
//Rev10 - DynamicJsonDoc moved from global to local- Removed OLED Displays, Updated Page 4 for battery voltage and maxLoopTime 
//Rev11 - Changed Pin assignments and migrated to LittleFS for internal ESP32-S3 flash logging.
          Changed data range to 0-1000
          At each power up two files are written to Internal Flash. 
          1) Settings file with all the settings at power up and
          2) Data file with engine operating data from power up till the ECU is powered off
          Added sd loop time and data file name info on Page 4.
          Added buttons on page 4 to reset error,manage files and change ECU modes. Added page 7 to be able to download and delete data files
          Added Fuel Solenoid capability. Added Two LED's to Show WiFi State and Recording state.
          Added ability to change ECU Webserver WiFi SSID and Password on page 1
          Added option for Magnetic (1 Pulse per revolution)  or Optical (2 pulses per revolution)  RPM sensor 
          implemented tempGradient check to limit sharp rise in exhaust temperature (tempGrad and maxTempGrad variables)
          Also implemented lookahead for temperature and stop increasing fuel flow if exhaust temp will increase above maxTemp in 3 sec (tempGrad x 3)
 Ardu-ECU
 ESP32 based ECU for model jet engines
/////////////////////////////////////////////////////////////////////////////////////////////////////////
 Disclaimer:
 Jet engine is not a toy and ECU control of a jet engine presents inherent risk.Any programming error, selection 
 of wrong parameters or component failure can cause damage to jet engine or other equipment and serious injury or
 death to people . I take no responsibility for any consequences for using this software. 
 Any person using this software uses it at their own risk and is responsible for all consequences intentional or otherwise.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
 Jehanzeb Khan
 Jehanzeb@digipak.org

 //Fault Handling
 Startup sequence - No ignition after gas on for given time--Handled with AbortAll()
 Startup sequence- No RPM increase with fuel pump running--handled with AbortAll()
 RPMAvg> Max RPM - Handled by reducing Fuel Flow
 exTemp>maxTemp - Handled by reducing Fuel Flow
 Flameout during operation - Handled with AbortAll()
 No RC Signal - Handled with AbortAll()
 No RPM Signal Handled with AbortAll()

 //Button Functions
 double click toggles between different modes e.g. Starter motor control and Fuel pump control and Trial modes. this is used to prime the fuel pump or purge the engine
Button pressed for 3 seconds will reset error codes and engine can be restarted after a fault. On Power up ECU will start a wifi web server which can be connected using phone or computer. address is 192.168.4.1
This is used to change ECU settings. SSID is "Ardu_ECU" and password is "admin123"

//Startup Sequence
Note: All settings in this sequence are changeable through web interface
1- Purge for x seconds at xx% Throttle to clear any combustible gases prior to starting 
2- Switch on Glow Plug and cycle between Ignition Low and High RPM  with gas valve open until ignition is achieved or timeout is reached
3- Confirm ignition if exhaust temperature crosses a limit(100 deg C default)
4- After ignition increase starter motor power as well as Fuel Flow. Chase the starting temperature(600 deg C default)  by increasing fuel flow and starter RPM
5- Switch off Gas, starter motor and glow plug as certain RPM are reached with fuel ( All programmable)
6- Keep increasing RPM with fuel flow until reach idle RPM.
7- Switch to idle mode and follow Throttle Input( maintain Idle RPM if throttle is zero)
8- Switch to operation mode and follow Throttle input

*/

// Libraries
#include <soc/pcnt_struct.h>
#include <driver/pcnt.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>//EEPROM library
#include "RunningAverage.h"
#include <LittleFS.h>
#include <MAX31855.h>
#include <ESP32Servo.h> 
#include <EasyButton.h>
#include <LiquidCrystal_I2C.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "page1.h"// html code for parameters screen
#include "page2.h"// html code for parameters screen
#include "page3.h"//html code for realtime data terminal screen 1
#include "page4.h"//html code for realtime data terminal screen 2
#include "page5.h"//html code for realtime data terminal screen 3
#include "page6.h"//Readings at abort time screen
#include "page7.h"//Engine and ECU Lifetime usage
#include "page8.h"//Data file download page


//for storing data to Internal Flash

TaskHandle_t fsTask; //Flash write Task to run on core 0 to avoid delay in main code
//Flash File
File file;
int fileseq=0;//Sequence in filename
char filename[32]; //Filename for data storage
char settingsfilename[32];//filename for settings data
//downloading and deleting files
String   webpage, MessageLine;
bool dirpresent=false;//if default data directory is present or not
bool fsavailable=false;
bool writefile=true; //only write file if this variable is true
String Data="";//Data string to be logged to Flash (written on core 0 only - see Task1code)
volatile bool headerPending=false;//header queued by core 1 (initLittleFS), written by core 0 (Task1code)
volatile bool sysMsgPending=false;//error message queued by core 1 (AbortAll), formatted by core 0 (SaveData)
volatile int sysMsgErrCode=0;//error code carried with the pending message

//I2C 16x2 LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);      //default PCF8574 address 0x27, 16 cols x 2 rows
char lcdLine1[17]="";                    //last written line 1 (for change detection)
char lcdLine2[17]="";                    //last written line 2

// --- Sensor & Analog Inputs (ADC1) ---
#define RC_Throttle_Pin      4
#define RC_Mode_Pin          5
#define voltsensorPin        6
#define RPMsensorPin         7 

// --- Thermocouple (SPI) ---
#define MAXCS                2   
#define ThermoSCK            39
#define ThermoSD             40

// --- Display & Interface ---
#define Button_Pin           8
#define Display_SDA          13
#define Display_SCL          14

// --- Core Engine Control ---
#define GasPin               15
#define Fuel_Solenoid_Pin    16
#define FuelPumpPin          17
#define GlowPin              18
#define StarterPin           21

// --- Switch & Potentiometer Control ---
#define EngineSwitch_Pin     10   //HIGH = run engine (switch+pot control), LOW = RC control
#define ThrottlePot_Pin       9   //potentiometer throttle 0-100% (used when EngineSwitch_Pin is HIGH)


//counter for RPM
#define PCNT_FREQ_UNIT      PCNT_UNIT_0                      // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)
pcnt_isr_handle_t user_isr_handle = NULL;                    // interrupt handler - not used
hw_timer_t * timer = NULL;                                   // Instancia do timer                                                              // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html

int16_t PulseCounter =     0;                                // pulse counter, max. value is 65536
int OverflowCounter =      0;                                // pulse counter overflow counter
int PCNT_H_LIM_VAL =       10000;                            // upper limit of counting  max. 32767, write +1 to overflow counter, when reached 
uint16_t PCNT_FILTER_VAL=  1000;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023


//defining push button function
EasyButton buttonA(Button_Pin);
bool pumpPrime=false;//variable to determine if throttle spins the starter motor or primes the pump in modeWaiting
//DNS Server
DNSServer dnsServer;
//Web server to change settings
AsyncWebServer server(80);
// Create an Event Source on /events using SSE(Server Sent Events) to update data in Data Terminal
AsyncEventSource events("/events");

bool startServer=true;//WiFi server should be started at powerup
bool serverStarted=false;// WiFi server has not started yet
bool eventServer=false;

String ssid     = "Ardu_ECU";
String password = "admin123";//Pass phrase has to be 8 or more characters for ssid to work
char* PARAM_ServerState = "state";
char* PARAM_maxTemp = "maxTemp";                           //Maximum exhaust temperature in degC. Engine will cut throttle if this temperature is exceeded. It will not shut down
char* PARAM_maxRPM = "maxRPM";                             //Maximum RPM. Engine will cut throttle if this is exceeded, it will not shut down the engine
char* PARAM_cpr = "cpr";                                     //Counts per revolution based on sensor type
char* PARAM_idleRPM = "idleRPM";                           //RPM while running at zero throttle. ECU will try to maintain this RPM at zero throttle
char* PARAM_rpmTolerance = "rpmTolerance";                      //RPM tolerance range, target RPMs will be RPM+ within rpmTolerance range
char* PARAM_glowOnRPM = "glowOnRPM";                       //RPM at which glow plug will be switched on at startup
char* PARAM_glowOffRPM = "glowOffRPM";                     //RPM exceeding which the glow plug will be turned off
char* PARAM_ignitionRPMHigh = "ignitionRPMHigh";           //At start up ECU will run engine between ignitionRPMHIGH and ignitionRPMLOW with gas and glow plug energized to create ignition
char* PARAM_ignitionRPMLow = "ignitionRPMLow";             //At start up ECU will run engine between ignitionRPMHIGH and ignitionRPMLOW with gas and glow plug energized to create ignition
char* PARAM_gasOnRPM= "gasOnRPM";                          //RPM at which gas valve will be turned On
char* PARAM_gasOffRPM = "gasOffRPM";                       //RPM at which gas valve will be turned off
char* PARAM_starterOffRPM = "starterOffRPM";               //RPM at which starter motor will be turned off
char* PARAM_fuelOnRPM = "fuelOnRPM";                       //RPM at which fuel pump will be switched on
char* PARAM_pumpOnValue = "pumpOnValue";                  //Initial value of fuel pump as fuel pumps donot start pumping until a certain input value
char* PARAM_purgeTime = "purgeTime";                       //Time for which engine will run at purge throttle without fuel or gas to clear the engine at begining
char* PARAM_purgeThrottle = "purgeThrottle";               //Throttle for purge operation in begining
char* PARAM_starterIncDelay = "starterIncDelay";           //Delay built into starter control to limit the rate of change of starter RPM
char* PARAM_starterLimit="starterLimit";                 //Used for brushless starter to stop at Throttle=starterLimit
char* PARAM_accelerationDelay = "accelerationDelay";       //Delay built in to limit fuel flow increase to avoid dumping too much fuel into engine
char* PARAM_decelerationDelay = "decelerationDelay";       //Delay built in to limit fuel flow decrease to avoid flameout by starving the engine
char* PARAM_lowDecelDelay = "lowDecelDelay";       //Delay built in to limit fuel flow decrease to avoid flameout by starving the engine
char* PARAM_lowAccelDelay = "lowAccelDelay";       //Delay built in to limit fuel flow decrease to avoid flameout by starving the engine
char* PARAM_ignitionThreshold = "ignitionThreshold";       //Temperture above which ignition will be detected
char* PARAM_noIgnitionThreshold="noIgnitionThreshold";     //Time in milliseconds after which startup sequence will be aborted due to no ignition
char* PARAM_fuelFlowDetectionTime="fuelFlowDetectionTime"; //Time in which engine should reach a certain RPM after starting fuel pump (Startup only)
char* PARAM_fuelFlowDetectionRPM="fuelFlowDetectionRPM";   //RPM to reach to confirm that engine is running on fuel and not on gas only
char* PARAM_trialModeFuelOnRPM="trialModeFuelOnRPM";   //RPM to reach to when fuel will be switched on for lubricating the bearings
char* PARAM_trialModeFuelFlow="trialModeFuelFlow";     //Fuel Flow to pump in Starter Only test mode to lubricate bearings
char* PARAM_startingTemp="startingTemp";                  //maximum temperature for starting phase
char* PARAM_maxTempGrad="maxTempGrad";                  //maximum temperature for starting phase
char* PARAM_maxMotorVolt="maxMotorVolt";                  //maximum motor voltage. this allows motors with less voltage limit to be used with bigger battery packs
char* PARAM_maxPumpVolt="maxPumpVolt";                  //maximum fuel pump voltage. this allows fuel pumps with less voltage limit to be used with bigger battery packs
char* PARAM_MIN_MICROS = "MIN_MICROS";              //minimum microseconds for servo control
char* PARAM_MAX_MICROS = "MAX_MICROS";                //maximum microseconds for servo control
char* PARAM_voltIn = "voltIn";                //Input voltage
char* PARAM_WiFiSSID = "WiFiSSID";                //Wifi Name/SSID
char* PARAM_WiFiPWD = "WiFiPWD";                //WiFi Password

 
String inputmaxTemp;
String inputmaxRPM;
String inputcpr;
String inputidleRPM;
String inputrpmTolerance;
String inputglowOnRPM ;
String inputglowOffRPM;
String inputignitionRPMHigh ;
String inputignitionRPMLow ;
String inputgasOnRPM;
String inputgasOffRPM ;
String inputstarterOffRPM ;
String inputfuelOnRPM ;
String inputpumpOnValue ;
String inputpurgeTime ;
String inputpurgeThrottle;
String inputstarterIncDelay;
String inputstarterLimit;
String inputaccelerationDelay;
String inputdecelerationDelay;
String inputlowAccelDelay;
String inputlowDecelDelay;
String inputignitionThreshold;
String inputnoIgnitionThreshold;
String inputfuelFlowDetectionTime;
String inputfuelFlowDetectionRPM;
String inputtrialModeFuelOnRPM;
String inputtrialModeFuelFlow;
String inputstartingTemp;
String inputmaxTempGrad;
String inputmaxMotorVolt;
String inputmaxPumpVolt;
String inputMIN_MICROS ;
String inputMAX_MICROS ;
String inputvoltIn ;
String inputWiFiSSID ;
String inputWiFiPWD ;

//Output Servo PPM Signal min and max times in microseconds
int MIN_MICROS=      1000;  
int MAX_MICROS=      2450;

// minimum and maximum values of output used in calculations. This will be scaled to servo min and max
int outMax=1000;
int outMin=0;
//output signal is identical to rc servo control signal and is generated using servo library
Servo fuelServo;
Servo startServo;

MAX31855 myMAX31855(MAXCS); //chip select pin

int exTemp=0;//Exhaust Temperature degC
int ambTemp=0;//Ambient Temperature degC
int tempGrad=0; //Temp change measured over 1 sec
int tempGradAvg=0; //Temp change averaged measured over 1 sec
int previousTemp=0;//previous value for measuring temperature gradient
long tempTimeOld=0;//to store previous measurement time

//engine modes during normal operation
#define modeWaiting       0  //No RPM //AutoStart
#define modeStarting      1  //RPM 0-Idle RPM  //Initiated through mode switch
#define modeIdling        2  //RPM IdleRPM
#define modeOperating     3  //RPM IdleRPM-MaxRPM
#define modeCooldown      4  //RPM IdleRPM-0

//Test Modes
#define modeStarterOnly   5  //Throttle controls the starter only- used to test the RPM limit starter can go without fuel
                              //mode control switch will control the gas flow
#define modeFuelPumpOnly  6  //Fuel pump is controlled through throttle control and not in firmware.Mode switch initiates startup

// Test mode variables
int trialModeFuelOnRPM = 6000; // RPM at which fuel flow will start  in trial mode
int trialModeFuelFlow = 0.1*outMax; //  Maximum fuel flow in starter only and Fuel Pump only trial mode(Should not be zero to avoid running dry bearings)

                              
//startup stages during modeStarting
#define startStage0   0  //Purge cycle to purge engine of any residual gases or fuel
#define startStage1   1  //Between Ignition and idle

int startStage=startStage0;  //At startup start with engine purge


bool RPMIncrease=false;//variable to cause starter to oscillate betweem ignitionRPMHigh and ignitionRPMLow until ignition 
#define serverLoopTime  300// update web server data terminal every 300ms
#define fsLoopTime   200 //snapshot of current data is written to Flash every 200 ms
#define rpmLoopTime   100  //RPM is calculated every 100ms
#define tempLoopTime  100  //Temp is measured every 100ms as that is time for max31855 to acquire one reading
#define rcLoopTime   20  //RC Signal is calculated every 20ms
#define voltageLoopTime 500 //battery voltage is measured every half second
#define saveUsageTime 60000 //Save engine usage data every minute
#define fsMaxTime 35  //if Flash write takes more than 35ms for fsMaxCount in a row stop writing
#define displayLoopTime 1000 //LCD display is refreshed once per second

long fsLoopTimeOld=0; //to store last time Flash was updated
long voltageLoopTimeOld=0; //to store last time battery voltage was updated
long displayLoopTimeOld=0; //to store last time LCD display was updated
long startStageTimeOld=0; //To keep track of time elapsed during start stage
long outputMillisOld=0;// variable to store millis value for calculating accel/deccel delay
long serverLoopTimeOld=0;//variable to keep track of web server data terminal update
long keepalivecounter=0; //variable to keep track of time to send live data to html dataterminal
long engineUsageTimeOld=0; //variable to keep track of engine running time
int engineMode=modeWaiting;  //Start with engine mode waiting
int startCount=0;   // ECU Restart count
int engineStartCount=0; //Engine start cycles
int engineRunTime = 0; //Engine Run Time in minutes



//RC Signal Variables
unsigned long rcThrottleSignal=0; //Throttle control
unsigned long rcModeSignal=0;  //Used for startup and cool down

//Switch & Potentiometer Control Variables
bool engineSwitchOn=false;    //engine switch (IO10) state: HIGH = switch+pot control, LOW = RC control
bool engineSwitchPrev=false;  //previous switch state for edge detection
bool switchControlActive=false;//engine was started/controlled via the switch (disables RC signal lost fault)
bool switchRestartArmed=true; //false after an abort: switch control must be re-cycled (or pot moved) before restart
bool potMovedAfterAbort=false;//true when the pot has moved away from idle since the last abort (used to re-arm restart)
int switchStableCount=0;      //debounce counter for engine switch (3 stable reads = 60ms)
int throttlePotValue=0;       //potentiometer throttle 0-100



// Engine Parameters
    // RPM Set points
int glowOnRPM=           1000;    //Glow plug is switched on as RPM increase above this point
int gasOnRPM=            2000;  //Switch on Gas when this RPM is exceeded
int ignitionRPMLow=      2200;   //ignition cycle low RPM
int ignitionRPMHigh=     6000;  //ignition cycle high RPM
int rpmTolerance=        5000;   //rpm Tolerance for target RPM (applicable at RPM higher than idle RPM only)
int fuelOnRPM=           6000;  //should be much lower than gasOffRPM to ensure continuous combustion
int pumpOnValue=        50;  //This is the value at which pump will start pumping fuel and is usually higher than zero
int glowOffRPM=          26000;  //Glow Plug switched off above this RPM
int gasOffRPM=           26000; //Gas is switched off and engine is accelerating on Fuel only
int starterOffRPM=       24000; //Started is switched of and engine is accelerating on Fuel and/or Gas only
int starterOffTime=      3000; //Starter will turn off if RPM remains above starterOffRPM for starterOffTime
int starterLimit=         0;  //Starter Throttle Limit for brushless starter
int idleRPM =            32000; //Idle RPM at zero Throttle
int maxRPM=              110000;//Never exceed RPM
int maxTemp=             800;   //Never exceed temperature
int maxTempGrad=         200;  //maximum temperature gradient (degC/Sec)
int startingTemp=        500;   //exhaust temperature maintained during startup
int tempTolerance=        50;   //temperature tolerance for fuel pump control.Not applicable at maxTemp
int noIgnitionThreshold=6000; //10000ms to abort due to no ignition
int ignitionThreshold=100; //if exTemp is higher than this value ignition is true otherwise ignition is false;
int purgeTime= 3000; //purge for 3 seconds
int purgeThrottle= (0.15*outMax); //purge Throttle is 20% of max
int starterIncDelay=5; //5ms between each iteration of starter motor control
int starterTransition=(0.1*outMax);//After this value starter delay will apply (outMin-outMax)
int accelerationDelay=60 ;  //delay time in millisecond for each iteration of output fuelpump control (While Accelerating)
int decelerationDelay=80; //delay time in millisecond for each iteration of output fuelpump control (While Decelerating)
int lowAccelDelay=600;  //delay time in millisecond for each iteration of output fuelpump control (While Accelerating)
int lowDecelDelay=500; //delay time in millisecond for each iteration of output fuelpump control (While Decelerating)
int fuelFlowDetectionTime=10000;  //time in millisecond after switching on fuel pump
int fuelFlowDetectionRPM=18000;  //RPM to confirm engine is running on Fuel
float maxMotorVolt=12.0;        //default max motor voltage is for 3 cell Lipo
float maxPumpVolt=12.0;        //default max fuel pump voltage is for 3 cell Lipo
bool brushedMotor=false; //i.e use startup logic for brushless motor -donot rely on RPM measurement which is incorrect with brushless starter


//RPM Variables
int          cpr=                   1; //counts per revolution. For Hall type sensors it is 1 and for through hole optical sensors it is 2 counts per revolution
volatile int revs=                 0;
volatile int RPM=                  0;
volatile int RPMAvg=               0;

int rpmGrad    =          0   ; //measured RPM gradient in RPM/Sec
int rpmGradAvg=0; //rpm change averaged measured over 1 sec
int previousRPM=0;//previous value for measuring RPM gradient


//constants
int RPMHoldGrad=          500; //RPM control will react if rpmGrad is higher than this value
int RPMRiseGrad=         20000;  //RPM Rise rate of 20000 RPM/Sec
int RPMDropGrad=         -10000;  //RPM Drop rate of 10000 RPM/Sec

// Other variables

volatile long measuredLoopTime=0;//main loop time measurement
volatile long fsWriteTime=0;
volatile long maxLoopTime=0;
int fsMaxCount=0; // no of times fsWrite time has exceeded max limit
volatile byte errorCode=0; //1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
bool ignitionState=false;//default state



RunningAverage RPMBuff(4);
RunningAverage RPMGradBuff(4);
RunningAverage tempGradBuff(4);

unsigned long RPMLoopTimeOld=0; //to store previous RPM calculation  time
unsigned long RCLoopTimeOld=0; //to start RC Signal measurement after this interval
unsigned long mainLoopTimeOld=0;//for measuring main loop time
unsigned long starterOffTimeOld=0;//for measuring starter time at max RPM
volatile unsigned long gasOnTime=0;//Time when gas valve was opened
volatile unsigned long RCModemicrosold=0;  //for calculating RC mode signal
volatile unsigned long RCThrottlemicrosold=0;//for calculating RC throttle signal
volatile unsigned long RCModemicros=0;
volatile unsigned long RCThrottlemicros=0;
volatile long RCSignalmillis=0;//To store time when RC signal was measured so RC signal failure can be detected
int RCSignalMaxTime=1000; //there should be RC signal within 1000 milliseconds
int rpmDiff=0; //RPM difference between current RPM and IDLE RPM

//Output Servo signal variables
int fuelFlowTarget=outMin; //Target value for fuel pump  
int fuelFlowNow=outMin; //current value of output signal for fuel pump  
long fuelFlowTimeOld=0; //Time of previous update for fuel pump 
int startMotorTarget=outMin;// target value for starter motor 
int startMotorNow=outMin;// current value of output signal for starter motor 
long startMotorTimeOld=0;// Time of previous update for starter motor 
int idleFuelFlow=outMin; //pump throttle at idle RPM  
bool gasFlow=false; //Solenoid gas valve for starting
bool fuelFlow=false;//Solenoid Fuel Valve
int glowPower=outMin; //output signal for glow plug  

//Active Valve control (v11_ActiveValve variant)
#define activeValvePeriod 1000      //solenoid pulse period in ms (max frequency is 1 Hz; the solenoid never switches more than once per second)
bool  activeValveActive=false;      //true when pulsed solenoid control is in use (flow below pump minimum)
float activeValveDuty=0.0;          //solenoid on-time duty cycle 0..1 for the current period
bool  activeValveOn=false;          //current solenoid state within the pulse period
unsigned long activeValveTimeOld=0; //millis() at the start of the current pulse period




//Voltage Sensor variables
   
int voltsensorValue = 0;  // variable to store the value coming from the sensor
float resistorRatio=7.1/1.5; //use actual values of resistor divider network as measured by multimeter to avoid error from resistor tolerance
float voltCorrection=1; //correction factor applied to resistance ratio to get accurate voltage measurement
float voltage=0;//Calculated voltage value
//Variables to store data at Abort condition

int abRPMAvg=0;
int abexTemp=0;
int abfuelFlowTarget=0; //output signal for fuel pump  0-1000
int abfuelFlowNow=0; //output signal for fuel pump  0-1000
int abstartMotorTarget=0;// output signal for starter motor 0-1000
int abstartMotorNow=0;// output signal for starter motor 0-1000
bool abgasFlow=false; //gas valve for starting
bool abfuelFlow=false; //gas valve for starting
bool abignitionState=false;
int abrcModeSignal=0;
int abrcThrottleSignal=0;
int abglowPower=0;//Stop Glow plug if On
int abengineMode=0;
int abstartStage=0;
float  abvoltage=voltage;
byte  aberrorCode=errorCode;

// --- Serial command/telemetry control (GUI interface) --- //
bool serialCmdActive=false;       //true when serial GUI override of RC inputs is active
int  serialCmdThrottle=0;         //serial throttle override 0-100
bool serialCmdThrottleValid=false;//set true by THROTTLE/START commands
int  serialCmdMode=100;           //serial mode override 0-100
bool serialCmdModeValid=false;    //set true by MODE signal override (START/STOP)
int  serialCmdRPM=0;              //simulated RPM override (GUI acts as virtual engine)
bool serialCmdRPMValid=false;     //set true by SETRPM
int  serialCmdTemp=0;             //simulated exhaust temp override (GUI acts as virtual engine)
bool serialCmdTempValid=false;    //set true by SETTEMP
unsigned long telemetryLoopTimeOld=0;
unsigned long telemetryLoopTime=1000;
String serialCmdBuf="";            //incoming serial line accumulator (non-blocking parser)
unsigned long serialCmdLastMillis=0;//time of last received serial line (watchdog)
unsigned long serialCmdTimeout=5000; //ms without serial traffic before reverting to physical control
unsigned long errorResetTime=0;    //timestamp for non-blocking error auto-reset in WaitingFunction

// --- Helper Functions required for LittleFS Replacement --- //
bool fsAvailableFlag=false;   //set true once LittleFS is mounted (set in initLittleFS)
bool fsWarned=false;          //only print the mount/open warning once

void appendFile(fs::FS &fs, const char * path, const char * message) {
    if (!fsAvailableFlag) {
        if (!fsWarned) { Serial.println("LittleFS not mounted - skipping file append"); fsWarned=true; }
        return;
    }
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        if (!fsWarned) { Serial.println("Failed to open file for appending"); fsWarned=true; }
        return;
    }
    file.print(message);
    file.close();
}

void LittleFS_deleteAll() {
    LittleFS.format();
}

void LittleFS_file_delete(String filename) {
    LittleFS.remove(filename);
}

void saveSettings() {
    File sFile = LittleFS.open(settingsfilename, FILE_WRITE);
    if(sFile){
        sFile.println(getSensorReadings()); 
        sFile.close();
    }
}
// --------------------------------------------------------- //

//RPM Counter
 void IRAM_ATTR CounterOverflow(void *arg) {                  // Interrupt for overflow of pulse counter
    OverflowCounter = OverflowCounter + 1;                     // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                    // clean overflow flag
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

void IRAM_ATTR isr_2() {  //RC Mode Signal Measurement
 
 if (digitalRead(RC_Mode_Pin)==HIGH) RCModemicrosold=micros();
 else  
 {
  if((micros()-RCModemicrosold)>800)RCModemicros=(micros()-RCModemicrosold);//ignore values below 800 microseconds
 }
  }

  void IRAM_ATTR isr_3() {  //RC Throttle Signal Measurement

 if (digitalRead(RC_Throttle_Pin)==HIGH) RCThrottlemicrosold=micros();
 else  
 {
  if((micros()-RCThrottlemicrosold)>800) RCThrottlemicros=(micros()-RCThrottlemicrosold);//ignore values below 800 microseconds
  RCSignalmillis=millis();//Updated at falling edge of RC Signal pulse
 }

 }

//  constants stored in EEPROM
Preferences preferences;

void setup(void) {
  
Serial.begin(115200);

//Initialize I2C 16x2 LCD display
Wire.begin(Display_SDA, Display_SCL);
lcd.init();
lcd.backlight();
lcd.clear();

// Initialize GPIO Pins

pinMode(Fuel_Solenoid_Pin,OUTPUT);
pinMode(GasPin,OUTPUT);
digitalWrite(Fuel_Solenoid_Pin,LOW);
digitalWrite(GasPin,LOW);

pinMode(RC_Mode_Pin,INPUT);
pinMode(RC_Throttle_Pin,INPUT);
pinMode(voltsensorPin,INPUT);

//Engine control switch (IO10) and throttle potentiometer (IO9)
pinMode(EngineSwitch_Pin, INPUT_PULLDOWN);//default LOW, switch pulls HIGH to start engine
pinMode(ThrottlePot_Pin, INPUT);

//Setting up button -On pressing for 3 seconds, it will reset error code. Also used for switching engine modes through doubleclicks
pinMode(Button_Pin, INPUT_PULLUP);
buttonA.begin();//Easy Button Library initialize
buttonA.onPressedFor(3000, BtnAFunc);//function to handle long press for 3 seconds
buttonA.onSequence(2, 1500, BtnADoubleClick);//handling double click

//RPM Setup
 pinMode(RPMsensorPin, INPUT);
initPulseCounter ();
 attachInterrupt(RC_Mode_Pin, isr_2, CHANGE);
 attachInterrupt(RC_Throttle_Pin, isr_3, CHANGE);

 RPMBuff.clear(); //clear the buffer used to average RPM values

myMAX31855.begin();

// Allow allocation of all timers
  //ESP32PWM::allocateTimer(0);//Timer 0 now used for pulse counter
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

    fuelServo.setPeriodHertz(50);// Standard 50hz servo
  startServo.setPeriodHertz(50);// Standard 50hz servo
  
  fuelServo.attach(FuelPumpPin, MIN_MICROS, MAX_MICROS);  
  startServo.attach(StarterPin, MIN_MICROS, MAX_MICROS);
  
  fuelServo.write(0);
  startServo.write(0);
  pinMode(GlowPin,OUTPUT);//glow plug driven as plain GPIO relay (no servo output)
  digitalWrite(GlowPin,LOW);
  delay(3000);//let the servo signals stabilize
  
  ReadSettings();//read constants from EEPROM if available
  //save ECU Start Count in EEPROM

preferences.begin("settings",false);
preferences.getInt("startCount",startCount);

startCount++;

preferences.putInt("startCount",startCount);
preferences.end();
Serial.print("ECU Powerup cycles  ");
Serial.println(startCount);   
 
//Initialize all timers/counters

  mainLoopTimeOld=serverLoopTimeOld=fsLoopTimeOld=RPMLoopTimeOld=RCLoopTimeOld=voltageLoopTimeOld=tempTimeOld=RCSignalmillis=millis();
//Create Task on core 0 to run Logging loop
xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "fsTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &fsTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  


}

//Parse serial commands from the GUI. Command names are case-insensitive.
//START | STOP | THROTTLE <0-100> | MODE <0-6> | ABORT | RESET | RC | PING | HELP
void ProcessSerialCommands()
{
  //Non-blocking parser: accumulate chars and only process complete lines,
  //so a partial line never blocks the control loop.
  while (Serial.available())
  {
    char ch=(char)Serial.read();
    if ((ch=='\n')||(ch=='\r'))
    {
      if (serialCmdBuf.length()>0)
      {
        ProcessSerialLine(serialCmdBuf);
        serialCmdBuf="";
      }
    }
    else if (serialCmdBuf.length()<64) serialCmdBuf+=ch;
  }
}

//Validate that arg is a non-negative integer string (rejects empty/garbage args).
bool serialArgIsNumber(String arg)
{
  if (arg.length()==0) return false;
  for (unsigned int i=0;i<arg.length();i++)
  {
    if ((arg[i]<'0')||(arg[i]>'9')) return false;
  }
  return true;
}

//Parse and execute one complete serial command line.
void ProcessSerialLine(String cmdLine)
{
  cmdLine.trim();
  if (cmdLine.length()==0) return;
  serialCmdLastMillis=millis();//any serial traffic keeps the override watchdog alive

  String cmd=cmdLine;
  String arg="";
  int sp=cmd.indexOf(' ');
  if (sp>0)
  {
    arg=cmd.substring(sp+1);
    cmd=cmd.substring(0,sp);
  }
  cmd.toUpperCase();
  if (cmd=="START")
  {
    serialCmdActive=true;
    serialCmdThrottle=0; serialCmdThrottleValid=true;
    serialCmdMode=100;   serialCmdModeValid=true;
    Serial.println("CMD:START OK");
  }
  else if (cmd=="STOP")
  {
    serialCmdActive=true;
    serialCmdMode=0; serialCmdModeValid=true;
    Serial.println("CMD:STOP OK");
  }
  else if (cmd=="THROTTLE")
  {
    if (!serialArgIsNumber(arg)) Serial.println("CMD:THROTTLE ERR");
    else
    {
      int n=arg.toInt();
      if (n<0) n=0;
      if (n>100) n=100;
      serialCmdActive=true;
      serialCmdThrottle=n; serialCmdThrottleValid=true;
      Serial.println(String("CMD:THROTTLE ")+String(n)+" OK");
    }
  }
  else if (cmd=="SETRPM")
  {
    if (!serialArgIsNumber(arg)) Serial.println("CMD:SETRPM ERR");
    else
    {
      long n=arg.toInt();
      if (n<0) n=0;
      if (n>100000) n=100000;
      serialCmdActive=true;
      serialCmdRPM=(int)n; serialCmdRPMValid=true;
      Serial.println(String("CMD:SETRPM ")+String(n)+" OK");
    }
  }
  else if (cmd=="SETTEMP")
  {
    if (!serialArgIsNumber(arg)) Serial.println("CMD:SETTEMP ERR");
    else
    {
      int n=arg.toInt();
      if (n<0) n=0;
      if (n>1000) n=1000;
      serialCmdActive=true;
      serialCmdTemp=n; serialCmdTempValid=true;
      Serial.println(String("CMD:SETTEMP ")+String(n)+" OK");
    }
  }
  else if (cmd=="MODE")
  {
    int n=arg.toInt();
    if ((serialArgIsNumber(arg))&&(n>=0)&&(n<=6))
    {
      engineMode=n;
      startStage=startStage0;
      Serial.println(String("CMD:MODE ")+String(n)+" OK");
    }
    else Serial.println("CMD:MODE ERR");
  }
  else if (cmd=="ABORT")
  {
    AbortAll();
    Serial.println("CMD:ABORT OK");
  }
  else if (cmd=="RESET")
  {
    errorCode=0;
    engineMode=modeWaiting;
    startStage=startStage0;
    Serial.println("CMD:RESET OK");
  }
  else if ((cmd=="RC")||(cmd=="REMOTE"))
  {
    serialCmdActive=false;
    serialCmdThrottleValid=false;
    serialCmdModeValid=false;
    serialCmdRPMValid=false;
    serialCmdTempValid=false;
    Serial.println("CMD:RC OK");
  }
  else if (cmd=="PING")
  {
    Serial.println("CMD:PING OK");
  }
  else if (cmd=="HELP")
  {
    Serial.println("CMD:START|STOP|THROTTLE n|MODE n|SETRPM n|SETTEMP n|ABORT|RESET|RC|PING|HELP");
  }
  else Serial.println("CMD:UNKNOWN "+cmdLine);
}

//Apply serial GUI override of RC inputs and simulated sensors when active
void ApplySerialOverride()
{
  if (serialCmdActive)
  {
    //Watchdog: if no serial traffic arrives for serialCmdTimeout ms the GUI
    //is assumed dead or disconnected, so revert to physical RC/switch control.
    if ((millis()-serialCmdLastMillis)>serialCmdTimeout)
    {
      serialCmdActive=false;
      serialCmdThrottleValid=false;
      serialCmdModeValid=false;
      serialCmdRPMValid=false;
      serialCmdTempValid=false;
      return;
    }
  }
  if (!serialCmdActive) return;
  switchControlActive=true;//suppress RC-signal-lost fault while serial override is active
  if (serialCmdThrottleValid) rcThrottleSignal=serialCmdThrottle;
  if (serialCmdModeValid) rcModeSignal=serialCmdMode;
  if (serialCmdRPMValid) RPMAvg=serialCmdRPM;
  if (serialCmdTempValid) exTemp=serialCmdTemp;
}

//One line of compact JSON telemetry, sent once per second
void SendSerialTelemetry()
{
  if ((millis()-telemetryLoopTimeOld)<telemetryLoopTime) return;
  telemetryLoopTimeOld=millis();

  const char* stage;
  if (engineMode==modeStarting) stage=(startStage==startStage0)?"purge":"ramp";
  else if (engineMode==modeIdling) stage="idle";
  else if (engineMode==modeOperating) stage="op";
  else if (engineMode==modeCooldown) stage="cool";
  else if (engineMode==modeStarterOnly) stage="t-st";
  else if (engineMode==modeFuelPumpOnly) stage="t-fuel";
  else stage="wait";

  Serial.print("{\"t\":");
  Serial.print(millis());
  Serial.print(",\"mode\":");
  Serial.print(engineMode);
  Serial.print(",\"stage\":\"");
  Serial.print(stage);
  Serial.print("\",\"thr\":");
  Serial.print(rcThrottleSignal);
  Serial.print(",\"modesig\":");
  Serial.print(rcModeSignal);
  Serial.print(",\"rpm\":");
  Serial.print(RPMAvg);
  Serial.print(",\"temp\":");
  Serial.print(exTemp);
  Serial.print(",\"volt\":");
  {
    float voltOut=voltage;
    if ((isnan(voltOut))||(voltOut<0)) voltOut=0;//keep JSON valid if the ADC reading is garbage
    if (voltOut>100) voltOut=100;
    Serial.print(voltOut,1);
  }
  Serial.print(",\"fuel\":");
  Serial.print(fuelFlowNow);
  Serial.print(",\"starter\":");
  Serial.print(startMotorNow);
  Serial.print(",\"glow\":");
  Serial.print(glowPower);
  Serial.print(",\"gas\":");
  Serial.print(gasFlow?1:0);
  Serial.print(",\"fuelv\":");
  Serial.print(fuelFlow?1:0);
  Serial.print(",\"err\":");
  Serial.print(errorCode);
  Serial.print(",\"run\":");
  Serial.print(engineRunTime);
  Serial.print(",\"loop\":");
  Serial.print(measuredLoopTime);
  Serial.println("}");
}

void loop(void) {
  
 if((startServer)&&(!serverStarted)&&(millis()>5000))//5 seconds before WiFi startup is given to allow power to stabilize
 {
   Serial.print("loop() running on core ");
  Serial.println(xPortGetCoreID());
  WebServerFunction();//Start webserver if 5 seconds have passed and WiFi server is not started
  initLittleFS();//Start Flash recording
 }
 
 if(serverStarted) dnsServer.processNextRequest();//process DNS request
   measuredLoopTime=millis()-mainLoopTimeOld;//calculate execution time of the program
   if (measuredLoopTime>maxLoopTime) maxLoopTime=measuredLoopTime;
   mainLoopTimeOld=millis();
  buttonA.read();
  CheckFailure();
  ReadRC();
  ReadRPM();
  ReadTempSensors(); //100ms loop time
  ReadVoltage();
  ApplySerialOverride();//serial GUI override of RC inputs and simulated sensors (if active)
  UpdateDisplay(); //LCD refresh once per second
  ProcessSerialCommands();//parse GUI commands from serial
  SendSerialTelemetry();//one line of JSON telemetry per second

//  if (errorCode==0)
// {
  switch(engineMode)
    {
      case modeWaiting:
      WaitingFunction();
      break;
      
      case modeStarting:
      StartingFunction();
      break;
      
      case modeIdling:
      IdlingFunction();
      break;
      
      case modeOperating:
      OperatingFunction();
      break;
      
      case modeCooldown:
      CooldownFunction();
      break;

      case modeStarterOnly://Test mode where only starter is controlled by throttle and gas is controlled by mode switch
      StarterOnlyFunction();
      break;

      case modeFuelPumpOnly://Test mode where only fuel pump is controlled by throttle and gas,glow and starter is controlled by mode switch
      FuelPumpOnlyFunction();
      break;
      
      default:
      WaitingFunction();
      break; 
//}
}
  ControlOutput();

  UpdateWebData();// Update web server with new data
  if (ignitionState) UpdateUsageData();
 
}

//Task1code: This is like a second loop functioning running on core0. By default main loop runs on Core 1 of ESP32
void Task1code( void * pvParameters ){
//This function will run forever, like a second loop function
  for(;;){      
   if(fsavailable && writefile) 
   {
    if (headerPending) { SaveDataHeader(); headerPending=false; }//header written here (core 0) so both Data users run on one core
    SaveData();
   }
        vTaskDelay(10 / portTICK_PERIOD_MS);
  } 
}
void UpdateWebData()
{
  
  if ((millis()-keepalivecounter)<5000)//Webdata is updated if a client is connected and counter is reset every 5 seconds
  {
  if((millis()-serverLoopTimeOld)>serverLoopTime)//If a client is connected , data is sent every 200 milli seconds 
{
    events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
    serverLoopTimeOld = millis();
}
}
}

//Update the I2C LCD. Runs once per second and only writes to the display when content changes.
void UpdateDisplay()
{
  if ((millis()-displayLoopTimeOld)<displayLoopTime) return;
  displayLoopTimeOld=millis();

  char line1[17], line2[17];
  int thr=(int)rcThrottleSignal;
  if (thr>100) thr=100;
  if (thr<0) thr=0;

  if (errorCode==0)
  {
    switch(engineMode)
    {
      case modeWaiting:        strcpy(line1,"Waiting");       break;
      case modeStarting:       strcpy(line1,(startStage==startStage0)?"Purge":"Ramp Up"); break;
      case modeIdling:         strcpy(line1,"Idling");        break;
      case modeOperating:      strcpy(line1,"Operating");     break;
      case modeCooldown:       strcpy(line1,"Cooldown");      break;
      case modeStarterOnly:    strcpy(line1,"Trial Starter"); break;
      case modeFuelPumpOnly:   strcpy(line1,"Trial Fuel");    break;
      default:                 strcpy(line1,"Unknown");       break;
    }
  }
  else if (bitRead(errorCode,0)) strcpy(line1,"No Ignition");
  else if (bitRead(errorCode,1)) strcpy(line1,"Temp Exceeded");
  else if (bitRead(errorCode,2)) strcpy(line1,"RPM Exceeded");
  else if (bitRead(errorCode,3)) strcpy(line1,"RC Signal Lost");
  else if (bitRead(errorCode,4)) strcpy(line1,"Flameout");
  else if (bitRead(errorCode,5)) strcpy(line1,"RPM Sensor Fail");
  else if (bitRead(errorCode,6)) strcpy(line1,"No Fuel Accel");
  else if (bitRead(errorCode,7)) strcpy(line1,"No Idle RPM");
  else                           strcpy(line1,"Error");

  snprintf(line2,17,"%d RPM %d%%",RPMAvg,thr);

  if (strcmp(line1,lcdLine1)!=0)
  {
    lcd.setCursor(0,0);
    lcd.print(line1);//print then pad to clear any leftover characters from previous longer text
    for(int i=strlen(line1);i<16;i++) lcd.print(" ");
    strcpy(lcdLine1,line1);
  }
  if (strcmp(line2,lcdLine2)!=0)
  {
    lcd.setCursor(0,1);
    lcd.print(line2);//print then pad to clear any leftover characters from previous longer text
    for(int i=strlen(line2);i<16;i++) lcd.print(" ");
    strcpy(lcdLine2,line2);
  }
}

void ReadVoltage()
{ 
  if((millis()-voltageLoopTimeOld)>voltageLoopTime)
{
  // read the value from the sensor:
   voltsensorValue = analogRead(voltsensorPin);
 voltage= resistorRatio*voltCorrection*3.3*(float(voltsensorValue)/4095);//voltage factor is the actual ratio of voltage divider used to reduce voltage from battery to ESP32 input range of maximum 3.3v
 voltageLoopTimeOld=millis();
}
}
void ReadRPM()
{

if ((millis()-RPMLoopTimeOld)>=rpmLoopTime)
{
  int timeElapsed=millis()-RPMLoopTimeOld;
Read_Reset_PCNT();
RPMLoopTimeOld=millis();
 revs=PulseCounter;
 revs=revs/cpr; //normalizing revs according to sensor type

  // read the value from the sensor:
   
     RPM=(revs*60*1000)/(timeElapsed);//As pulse counter is operating on rising edge
     RPMBuff.addValue(RPM);
     RPMAvg= RPMBuff.getAverage();//we use average to smooth out RPM

     rpmGrad =  ((RPM-previousRPM)*1000)/timeElapsed; 
     RPMGradBuff.addValue(rpmGrad);
    rpmGradAvg=RPMGradBuff.getAverage(); 
    previousRPM=RPM;//previous value for measuring RPM gradient
  
}
}
void ReadRC()//values are updated in ISR and this function processes the result
{
  if ((millis()-RCLoopTimeOld)>=rcLoopTime)
  {
//saving previous values
unsigned long rcThrottleSignalold=rcThrottleSignal;
unsigned long rcModeSignalold=rcModeSignal;

//Read engine control switch (IO10). HIGH = switch+pot control, LOW = RC control.
//Debounced: a level change is accepted only after 3 consecutive stable reads (~60ms).
bool rawSwitch=digitalRead(EngineSwitch_Pin);
if (rawSwitch==engineSwitchOn) switchStableCount=0;//stable, reset debounce counter
else
{
  switchStableCount++;
  if (switchStableCount>=3)
  {
    engineSwitchPrev=engineSwitchOn;
    engineSwitchOn=rawSwitch;
    switchStableCount=0;
  }
}

if (engineSwitchOn)//switch pulled high: use switch + potentiometer for control
{
  switchControlActive=true;//engine is under switch+pot control, so ignore RC signal lost fault
  //Potentiometer (IO9) outputs 0-100 throttle, 0=idle, 100=maxRPM
  throttlePotValue=map(analogRead(ThrottlePot_Pin),0,4095,0,100);
  if (throttlePotValue>100) throttlePotValue=100;
  if (throttlePotValue<0) throttlePotValue=0;
  rcThrottleSignal=throttlePotValue;
  rcModeSignal=100;//force mode switch on so engine stays running
  //Re-arm restart safety when the pot is deliberately moved away from idle and back
  if (throttlePotValue>10) potMovedAfterAbort=true;
  else if ((potMovedAfterAbort)&&(throttlePotValue<10))
  {
    switchRestartArmed=true;
    potMovedAfterAbort=false;
  }
}
else//switch pulled low: use original RC input logic
{
  switchRestartArmed=true;//re-cycling the switch re-arms restart after an abort
  if (engineSwitchPrev)//one-shot falling edge: switch was high and user pulled it low
  {
    engineSwitchPrev=false;//consume the edge so it doesn't re-trigger
    if ((engineMode==modeStarting)||(engineMode==modeIdling)||(engineMode==modeOperating)) engineMode=modeCooldown;
  }
  if ((engineMode==modeWaiting)&&(!ignitionState)) switchControlActive=false;//back to RC control only when engine is cool
  rcThrottleSignal=RCThrottlemicros;
  if((rcThrottleSignal<1000)) rcThrottleSignal=1000;
  rcThrottleSignal=(rcThrottleSignal-1000)/10;//RC signal in %
  //removing signal glithes bigger than 30
  if (rcThrottleSignal>(rcThrottleSignalold+30))rcThrottleSignal=rcThrottleSignalold+30;
  if ((rcThrottleSignalold>30)&&(rcThrottleSignal<(rcThrottleSignalold-30)))rcThrottleSignal=rcThrottleSignalold-30;

  rcModeSignal=RCModemicros;
  if((rcModeSignal<1000)||(rcModeSignal>2300)) rcModeSignal=1000;
  rcModeSignal=(rcModeSignal-1000)/10;//RC signal in %

  //removing signal glithes bigger than 30
  if (rcModeSignal>(rcModeSignalold+30))rcModeSignal=rcModeSignalold+30;
  if ((rcModeSignalold>30)&&(rcModeSignal<(rcModeSignalold-30)))rcModeSignal=rcModeSignalold-30;
}
RCLoopTimeOld=millis();
  
  }
}


void ReadTempSensors()
{
  if ((millis()-tempTimeOld)>tempLoopTime)
  {
    digitalWrite(MAXCS,LOW);
  uint32_t rawData = myMAX31855.readRawData();
  exTemp = myMAX31855.getTemperature(rawData);
  ambTemp=myMAX31855.getColdJunctionTemperature(rawData);
   if (isnan(exTemp)) exTemp=0;
   int elapsedtime=millis()-tempTimeOld;
   tempGrad=(exTemp-previousTemp)*1000/elapsedtime;
   tempGradBuff.addValue(tempGrad);
   tempGradAvg= tempGradBuff.getAverage();//we use average to smooth out temperature gradient
   previousTemp=exTemp;
   tempTimeOld=millis();
 if ((ignitionState) &&(exTemp<ignitionThreshold))//detect flameout condition when after ignition temperature drops below threshold
   {
    if (fuelFlowNow>0){
     
    bitSet(errorCode,4);//1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
    AbortAll();
    }
    else{
      AbortAll();
         }
    
   }
  if (exTemp>=ignitionThreshold) ignitionState=true;
  if (exTemp<ignitionThreshold) ignitionState=false;
  digitalWrite(MAXCS,HIGH);
  }
 
}

  void ReadSettings()//Read settings from Flash memory
{  //name of the key in key-value pair should be maximum 15 characters
   preferences.begin("settings",true);
    maxRPM=preferences.getInt("maxRPM",maxRPM);
   maxTemp=preferences.getInt("maxTemp",maxTemp);
   idleRPM=preferences.getInt("idleRPM",idleRPM);
   rpmTolerance=preferences.getInt("rpmTolerance",rpmTolerance);
   glowOnRPM=preferences.getInt("glowOnRPM",glowOnRPM);
   glowOffRPM=preferences.getInt("glowOffRPM",glowOffRPM);
  ignitionRPMHigh=preferences.getInt("ignitionRPMHigh",ignitionRPMHigh);
    ignitionRPMLow=preferences.getInt("ignitionRPMLow",ignitionRPMLow);
      gasOnRPM=preferences.getInt("gasOnRPM",gasOnRPM);
   gasOffRPM=preferences.getInt("gasOffRPM",gasOffRPM);
   starterOffRPM=preferences.getInt("starterOffRPM",starterOffRPM);
   fuelOnRPM=preferences.getInt("fuelOnRPM",fuelOnRPM);
    pumpOnValue=preferences.getInt("pumpOnValue",pumpOnValue);
   purgeThrottle=preferences.getInt("purgeThrottle",purgeThrottle);
   purgeTime=preferences.getInt("purgeTime",purgeTime);
   starterIncDelay=preferences.getInt("starterIDelay",starterIncDelay);
      starterLimit=preferences.getInt("starterLimit",starterLimit);
  noIgnitionThreshold=preferences.getInt("noIgnThreshold",noIgnitionThreshold);//Time after which it is decided that starting gas has not ignited
  ignitionThreshold=preferences.getInt("ignThreshold",ignitionThreshold);//temperature at which ignition is detected
  accelerationDelay=preferences.getInt("accelDelay",accelerationDelay);
   decelerationDelay=preferences.getInt("decelDelay",decelerationDelay);
    fuelFlowDetectionTime=preferences.getInt("fuelDetTime",fuelFlowDetectionTime);
fuelFlowDetectionRPM=preferences.getInt("fuelDetRPM",fuelFlowDetectionRPM);
trialModeFuelOnRPM=preferences.getInt("trModeFuelOnRPM",trialModeFuelOnRPM);
trialModeFuelFlow=preferences.getInt("trModeFuelFlow",trialModeFuelFlow);
startingTemp=preferences.getInt("startingTemp",startingTemp);
maxTempGrad=preferences.getInt("maxTempGrad",maxTempGrad);
maxMotorVolt=preferences.getFloat("maxMotorVolt",maxMotorVolt);
maxPumpVolt=preferences.getFloat("maxPumpVolt",maxPumpVolt);
    MIN_MICROS=preferences.getInt("MIN_MICROS",MIN_MICROS);//Servo PPM Signal min and max times in microseconds
   MAX_MICROS=preferences.getInt("MAX_MICROS",MAX_MICROS);
   RPMRiseGrad=preferences.getInt("RPMRiseGrad",RPMRiseGrad);
   RPMDropGrad=preferences.getInt("RPMDropGrad",RPMDropGrad);
   lowAccelDelay=preferences.getInt("lowADelay",lowAccelDelay);
   lowDecelDelay=preferences.getInt("lowDDelay",lowDecelDelay);
   startCount=preferences.getInt("startCount",startCount);
   engineStartCount=preferences.getInt("engineStarts",engineStartCount);
   engineRunTime=preferences.getInt("runTime",engineRunTime);
 voltCorrection=preferences.getFloat("voltCorr",voltCorrection);
  ssid=preferences.getString("SSID",ssid);
   password=preferences.getString("PWD",password);
   cpr=preferences.getInt("cpr",cpr);//counts per revolution for RPM sensor
 preferences.end();

}   
void StarterOnlyFunction()//Test function where throttle will control starter 
{

 //Gas flow control
  if ((rcModeSignal>80)&&(glowPower==outMax))gasFlow=true;   //Switch on gas when mode switch is flipped on and glow plug is energized
   if ((RPMAvg>gasOffRPM)||(RPMAvg<gasOnRPM)) gasFlow=false;   
  if (rcModeSignal<20)
  { 
    //Donot use AbortAll() as we dont want mode to reset
    gasFlow=false;  //Switch off gas when mode switch is flipped off
    fuelFlow=false;//Switch off fuel valve
    glowPower=outMin;
    fuelFlowTarget=0;
    fuelFlowNow=0;
     startStageTimeOld=0;
      gasOnTime=0;
     
  }

 startMotorTarget=map(rcThrottleSignal,0,100,outMin,outMax);        //starter will follow throttle control

   //glow plug operation
    if ((RPMAvg>=glowOnRPM)&&(RPMAvg<glowOffRPM))glowPower=outMax;
    else glowPower=outMin;
   
 
//fuel flow control
    if ((ignitionState)&&(RPMAvg>=ignitionRPMHigh)&&(fuelFlowTarget<trialModeFuelFlow))
    {
   
      if ((startMotorTarget<(0.7*outMax))&&(RPMAvg<starterOffRPM))//If starter below 70% of max, maintain starting temp
      {
      if(exTemp<=startingTemp) fuelFlowTarget++;
     else  fuelFlowTarget--;
        }
        else  // if starter is maxed out use max temp to reach target fuel flow
     {
      if(exTemp<=maxTemp) fuelFlowTarget++;
     else fuelFlowTarget--;
        }
    }
     if ((ignitionState)&&(fuelFlowNow>trialModeFuelFlow)) fuelFlowTarget--; //In case fuel flow target is changed during operation through web interface
   if(!ignitionState){
    fuelFlow=false;//Switch off fuel solenoid
    fuelFlowTarget=outMin;
   }
}
void FuelPumpOnlyFunction()//Test function where throttle will control fuel pump
{
  //Abort control
   if (rcModeSignal<20) 
   {
    //Donot use AbortAll() as we dont want mode to reset
    gasFlow=false;  //Switch off gas when mode switch is flipped off
    fuelFlow=false;//Switch off fuel valve
    glowPower=outMin;
    fuelFlowTarget=0;
    fuelFlowNow=0;
     startStageTimeOld=0;
      gasOnTime=0;
       startMotorTarget=outMin;
      startMotorNow=outMin; //immediately stop the motor without delay
   }

   //Normal Operation
  if (rcModeSignal>80) 
  {
 //ignition RPM cycling
    if(!ignitionState)
    {
     if(RPMIncrease) startMotorTarget++;
     else {
      startMotorTarget=outMin;
      startMotorNow=outMin; //immediately stop the motor without delay
     }
      if (starterLimit==0){
        if((RPMAvg>ignitionRPMHigh)) RPMIncrease=false;//At ignitionRPMHigh start reducing starter RPM
     else if((RPMAvg<ignitionRPMLow)) RPMIncrease=true;//At ignitionRPMLow start increasing starter RPM
      }
      else
      {
         if(startMotorNow>(ignitionRPMHigh/100)) RPMIncrease=false;//At ignitionRPMHigh start reducing starter RPM
     else if((RPMAvg<ignitionRPMLow)) RPMIncrease=true;//At ignitionRPMLow start increasing starter RPM
      }
    }
    //glow plug operation
    
    if ((RPMAvg>=glowOnRPM)&&(RPMAvg<glowOffRPM))glowPower=outMax;
   else glowPower=outMin;

       //gas flow operation
    if ((glowPower==outMax)&&(gasFlow==false)&&(RPMAvg>gasOnRPM)&&(RPMAvg<gasOffRPM)) 
    {
      gasFlow=true;
      gasOnTime=millis();
    }
    if ((gasOnTime>0)&&((millis()-gasOnTime)>noIgnitionThreshold)&&(!ignitionState) )
    {
      AbortAll();
     bitSet(errorCode,0);//No Ignition //1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
      gasOnTime=0;
    }
     if ((starterLimit==0)&&((RPMAvg>gasOffRPM)||(RPMAvg<gasOnRPM))) gasFlow=false;

     //starter Motor Opertion
     //Starter switchoff logic
      if ((starterLimit==0)&&(RPMAvg>starterOffRPM)&&(exTemp<maxTemp))//With starter Limit set to zero, brushed motor is assumed and starter switches off at starterOffRPM
      {
       
          startMotorTarget=outMin;
          startMotorNow=outMin;
                   
       }
      else if((starterLimit>0)&&(startMotorNow>=starterLimit)&&(exTemp<maxTemp))//brushless motor will stop on starterLimit as RPM measurement with brushless starter is incorrect for magnetic RPM sensor
      {
          startMotorTarget=outMin;
          startMotorNow=outMin;
                
       }
   
   //     Accelerate with gas and starter till trialMode Fuel On RPM-while keeping exhaust temperature above ignition Threshold
    if (exTemp>(ignitionThreshold+tempTolerance)) //ignition is established and tolerance margin added to avoid going below ignition temp as soon as motor starts
       {   //Increase start motor once
       if(starterLimit==0)  //Brushed Motor- decision is made on RPM
       {
        if(RPMAvg<ignitionRPMHigh)  startMotorTarget++;//Increase RPM above ignitionRPMHigh as long as ignition is on
     else if((exTemp>startingTemp)&&(startMotorTarget<outMax)&&(RPMAvg<starterOffRPM)&&( startMotorNow>outMin)) startMotorTarget++;//Increase Starter RPM only if starting temp is exceeded
       }
       else //Brushless motor case decision is made on starter throttle and not on RPM
        {
          if(startMotorNow<ignitionRPMHigh/100)  startMotorTarget++;//Increase RPM above ignitionRPMHigh as long as ignition is on
             else if((exTemp>(startingTemp+tempTolerance))&&(startMotorTarget<outMax)&&(startMotorNow<starterLimit)&&( startMotorNow>outMin)) startMotorTarget++;//Increase Starter RPM only if starting temp is exceeded
         }
       }
        //Fuel Flow manual control once ignition is established. Fuel flow will stop increasing if any of these are exceeded  trialModeFuelFlow, maxRPM,maxTemp  
        if(ignitionState) 
        {
       
          if (pumpOnValue>outMin) fuelFlowTarget=pumpOnValue + int(float((rcThrottleSignal*trialModeFuelFlow)/100));//If ignition true then fuel pump will follow throttle signal
        else fuelFlowTarget= int(float((rcThrottleSignal*trialModeFuelFlow)/100));//If ignition true then fuel pump will follow throttle signal
        
        }
           else 
           {
            fuelFlow=false;//Switch off Fuel Solenoid
            fuelFlowTarget=outMin;
           }
              }        
}
void WaitingFunction()
{
if (errorCode>0)
{
  //Non-blocking error auto-reset: hold the fault for 3 s, then clear it,
  //without freezing the loop (telemetry and serial commands keep running).
  if (errorResetTime==0) errorResetTime=millis();
  if ((millis()-errorResetTime)>=3000)
  {
    errorCode=0; //reset error code
    errorResetTime=0;
  }
}
else errorResetTime=0;
 
  startStageTimeOld=0;
  gasOnTime=0;
  glowPower=outMin;
  gasFlow=false;
  fuelFlow=false;
  fuelFlowTarget=outMin;//This will change in below command if system is in pumpPrime mode
  
 if(!pumpPrime) startMotorTarget=map(rcThrottleSignal,0,100,outMin,outMax);        //starter will follow throttle control
else fuelFlowTarget=map(rcThrottleSignal,0,100,outMin,outMax);        //follow throttle control
if ((rcModeSignal>80)&&(rcThrottleSignal<10)&&(exTemp<ignitionThreshold))//Start only when throttle is below 10%
{
  if ((!engineSwitchOn)||(switchRestartArmed))//switch control must be re-armed after an abort; RC starts normally
  {
    pumpPrime=false;//reset throttle control at startup to starter control
    engineMode=modeStarting;
    startStage=startStage0;
  }
}

}
void StartingFunction()
{
  if (rcModeSignal<20)
  {
    if (switchControlActive) engineMode=modeCooldown;//switch pulled low during switch control: cool down
    else AbortAll();//original RC logic: abort
  }
  else
  {
 
    if (startStage==startStage0)  //Purge cycle
    {
      if (startStageTimeOld==0) startStageTimeOld=millis();//initialize the purge Timer
      if ((startStageTimeOld>0)&&((millis()-startStageTimeOld)>purgeTime))//determine if end of Purge cycle reached
      {
        startMotorTarget=outMin;
        
      }
       if ((startStageTimeOld>0)&&((millis()-startStageTimeOld)>purgeTime)&& (RPMAvg<ignitionRPMLow))//determine if end of Purge cycle reached
      {
       
        startStageTimeOld=0;
        startStage=startStage1;//move to next stage (ignition)
       
      }
      if ((startStageTimeOld>0)&&((millis()-startStageTimeOld)<purgeTime)) startMotorTarget=purgeThrottle;//set Start Motor at purge Throttle
     
    }
    else    // if (startStage==startStage1)  //ignition cycle
    {
     //glow plug operation
    if ((RPMAvg>=glowOnRPM)&&(RPMAvg<glowOffRPM))glowPower=outMax;
    else  glowPower=outMin;

    //gas flow operation
    if ((glowPower==outMax) &&(gasFlow==false)&&(RPMAvg>gasOnRPM)&&(RPMAvg<gasOffRPM)) 
    {
      gasFlow=true;
      gasOnTime=millis();
    }
    if ((gasOnTime>0)&&((millis()-gasOnTime)>noIgnitionThreshold)&&(!ignitionState) )
    {
      AbortAll();
       //1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
     bitSet(errorCode,0);//No Ignition
      gasOnTime=0;
    }
   
    if ((starterLimit==0)&&((RPMAvg>gasOffRPM)|| (RPMAvg<gasOnRPM)))
    {
      gasFlow=false;
      gasOnTime=0;
    }
  
    
    //ignition cycle -oscillate between two RPM's untill ignition is established
    if(!ignitionState)
    {
     if(RPMIncrease) startMotorTarget++;
     else 
     {
      startMotorTarget=outMin;//Using a target of zero has worked better as compared to gradually reducing RPM
     startMotorNow=outMin;
     }
      if (starterLimit==0){
        if((RPMAvg>ignitionRPMHigh)) RPMIncrease=false;//At ignitionRPMHigh start reducing starter RPM
     else if((RPMAvg<ignitionRPMLow)) RPMIncrease=true;//At ignitionRPMLow start increasing starter RPM
      }
      else //(Starter Limit > 0)
      {
         if(startMotorNow>(ignitionRPMHigh/100)) RPMIncrease=false;//using ignitionRPMHigh as a throttle control
     else if((RPMAvg<ignitionRPMLow)) RPMIncrease=true;//At ignitionRPMLow start increasing starter RPM
      }
    
    }
    
        //Ramp up to idle- starter Motor and fuel flow during ramp up Opertion
 
else//ignition is established
{
 engineUsageTimeOld=millis(); //initialize engine use counter
if(RPMAvg<ignitionRPMHigh) startMotorTarget++ ;  //first get to ignitionRPMHigh
else if ((RPMAvg<starterOffRPM)&&(starterLimit==0))//ignition is established and starter is not yet off and is brushed motor
{

  if (startMotorTarget<outMax)
      {
        //Ramp up starter RPM while keeping Temp at Starting Temp
        if(exTemp>startingTemp) 
        {
          startMotorTarget++;
          if ((exTemp+(2*tempGrad))>maxTemp) fuelFlowTarget--;    //2 sec look ahead
        }
        else if (tempGrad<=0)fuelFlowTarget++;
      }
    else //  if start motor is maxed out
       { 
        //Keep increasing fuel flow but keep temperature in check. 
        //If fuel flow increase temperature above max temp and RPM is below self sustaining RPM and starter motor is already at 100% 
        // then starter motor is not giving the RPM to reach self sustaining mode.Engine will not accelerate further and 
        //will not reach self sustaining RPM
        
        if (exTemp<maxTemp)fuelFlowTarget++;
             else fuelFlowTarget--;
      }
}
else if ((starterLimit==0)&&(RPMAvg>starterOffRPM))
      
{
  fuelFlowTarget++;//Give a little kick as starter will shut off and now it will switch to idling mode
   startMotorTarget=outMin;
   startMotorNow=outMin;
    engineMode=modeIdling; 
    engineStartCount++;  //Increment engine usage counter
}

else if (starterLimit>0)//brushless motor starter. RPM sensor is not sensed motor is used to max throttle and then stopped
{
 
  if (startMotorNow<starterLimit)
      {
        //Ramp up starter RPM while keeping Temp at Starting Temp
        if(exTemp>startingTemp)
        {
           startMotorTarget++;
          if ((exTemp+(2*tempGrad))>maxTemp) fuelFlowTarget--;    //2 sec look ahead
        
        }
        else if (tempGrad<=0)fuelFlowTarget++;
      }
     else if(exTemp>maxTemp) startMotorTarget++;//This will happen if startMotorNow => starterLimit and temp has exceeded max Temp
     else if ((exTemp<(maxTemp-100))&&(startMotorNow>(starterLimit+10))) startMotorNow=(starterLimit*0.8); //once temp drops below maxTemp, reset starter at 80% of starterLimit
    else //  if start motor is maxed out goto idle mode
       { 
          startMotorTarget=outMin;
          startMotorNow=outMin;
         engineMode=modeIdling; 
         engineStartCount++;//Increment engine usage counter
         
         }
        }
       }
      }
     }
    }
void IdlingFunction(){

  //for brushless motor we dont depend on RPM to shut down gas and glow plug in starting functions
  gasFlow=false;
  gasOnTime=0;
  glowPower=outMin;
  //Also switch off starter motor just in case
  startMotorTarget=outMin;
  startMotorNow=outMin;
   
  if (rcModeSignal<20) engineMode=modeCooldown;
  else{
int targetRPM=idleRPM+ int(round(((maxRPM-idleRPM)*rcThrottleSignal)/100));
if((RPMAvg<maxRPM)&&(RPMAvg<targetRPM)&&(exTemp<maxTemp)) fuelFlowTarget++;//only increase fuel flow if Average RPM< max RPM and exhaust temp<max Temp
else if ((RPMAvg>(targetRPM+rpmTolerance))||(RPMAvg>maxRPM)||(exTemp>maxTemp)) fuelFlowTarget--;
//else hold the current fuelFlowTarget - NOT snapped to fuelFlowNow, which is
//floored at pumpOnValue-1; snapping pins the target to the pump minimum and
//defeats sub-minimum active-valve pulsing in idle/operating
if (RPMAvg>(idleRPM+3000)) engineMode=modeOperating;
}
}
void OperatingFunction(){

if (rcModeSignal<20)
{
  if (switchControlActive) engineMode=modeCooldown;//switch pulled low during switch control: cool down
  else AbortAll();//original RC logic: abort
}
else
{
int targetRPM=idleRPM+ int(round(((maxRPM-idleRPM)*rcThrottleSignal)/100));
if((RPMAvg<maxRPM)&&(RPMAvg<targetRPM)&&(exTemp<maxTemp)) fuelFlowTarget++;//only increase fuel flow if Average RPM< max RPM and exhaust temp<max Temp
else if ((RPMAvg>(targetRPM+rpmTolerance))||(RPMAvg>maxRPM)||(exTemp>maxTemp)) fuelFlowTarget--;
//else hold the current fuelFlowTarget - NOT snapped to fuelFlowNow, which is
//floored at pumpOnValue-1; snapping pins the target to the pump minimum and
//defeats sub-minimum active-valve pulsing in idle/operating
if (RPMAvg<(idleRPM+3000)) engineMode=modeIdling;
}
}
void CooldownFunction(){
 if(fuelFlowTarget>outMin) fuelFlowTarget=outMin;//Shut down fuel flow

 if ((RPMAvg<12000)&&(exTemp>ignitionThreshold))//Run starter motor till temperature is below ignition temperature
 {
//   fuelFlowTarget=outMin;
   startMotorTarget=purgeThrottle;
 }

if (exTemp<ignitionThreshold)startStageTimeOld=millis();//cool down till ignition detection temperature and initialize the purge Timer
       if ((startStageTimeOld>0)&&((millis()-startStageTimeOld)>purgeTime))//determine if end of Purge cycle reached
 
      {
        startStageTimeOld=0;//Reset timer
       startMotorTarget=outMin;
       engineMode=modeWaiting;
       
      }

}
void CheckFailure()
{
  
 //1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
 
 if  (exTemp>maxTemp) 
 { 
  bitSet(errorCode,1);//Max Temp Exceeded
    if (rcModeSignal<20) AbortAll();
else  if (fuelFlowTarget>5)  fuelFlowTarget=fuelFlowTarget-5;
else{
  fuelFlow=false;
  fuelFlowTarget=outMin;
}
 }
 else bitClear(errorCode,1);//1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
 if (RPMAvg>maxRPM) 
 {
  bitSet(errorCode,2);//Max RPM Exceeded
  if (rcModeSignal<20) AbortAll();
 else if (fuelFlowTarget>5) fuelFlowTarget=fuelFlowTarget-5;
 else {
  fuelFlow=false;
  fuelFlowTarget=outMin;
 }
 }
  else bitClear(errorCode,2);

   //if no RC Signal within RCSignalMaxTime. Ignored when switch+pot control is active (no RC receiver used)
 if (switchControlActive) bitClear(errorCode,3);
 else if ((millis()>RCSignalmillis)&&((millis()-RCSignalmillis)>RCSignalMaxTime) )
 {
  if (ignitionState){
     //1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
  bitSet(errorCode,3);//No RC Signal
  AbortAll();}
 }
 else  bitClear(errorCode,3);
 //RPM Sensor Failure
 if (pumpOnValue>outMin)
 {
if((!pumpPrime)&&(RPMAvg==0)&&(fuelFlowTarget>pumpOnValue))
{
   bitSet(errorCode,5);//RPM Failure
  AbortAll();
}
 }
 else
 {
if((!pumpPrime)&&(RPMAvg==0)&&(fuelFlowTarget>outMin))
{
   bitSet(errorCode,5);//RPM Failure
  AbortAll();
}
 }

if((!pumpPrime)&&(RPMAvg==0)&&(startMotorNow>((outMax-outMin)/4)))//Throttle is higher than 25% of scale and RPM is zero
{
   bitSet(errorCode,5);//RPM Failure
  AbortAll();
}
if((engineMode==modeOperating)&&(RPMAvg<idleRPM))//RPM incorrect as per mode
{
   bitSet(errorCode,5);//RPM Failure
  AbortAll();
}
if((engineMode==modeIdling)&&(RPMAvg<(starterOffRPM-5000)))//RPM incorrect as per mode
{
   bitSet(errorCode,5);//RPM Failure
  AbortAll();
}
 
 }


void AbortAll()
{ 
  sysMsgErrCode=errorCode;
  sysMsgPending=true;//core 1 -> core 0 flag; SaveData formats the text (no cross-core String access)
  //take snapshot of critical data for analysis
  abRPMAvg=RPMAvg;
  abexTemp=exTemp;
  abfuelFlowTarget=fuelFlowTarget; //output signal for fuel pump  0-1000
  abfuelFlowNow=fuelFlowNow; //output signal for fuel pump  0-1000
  abstartMotorTarget=startMotorTarget;// output signal for starter motor 0-1000
  abstartMotorNow=startMotorNow;// output signal for starter motor 0-1000
  abgasFlow=gasFlow; //gas valve for starting
  abfuelFlow=fuelFlow; //fuel valve for starting
  abignitionState=ignitionState;
  abrcModeSignal=rcModeSignal;
  abrcThrottleSignal=rcThrottleSignal;
  abglowPower=glowPower;//Stop Glow plug if On
  abengineMode=engineMode;
  abstartStage=startStage;
  abvoltage=voltage;
  aberrorCode=errorCode;

  
  fuelFlowTarget=outMin; //output signal for fuel pump  0-1000
  fuelFlowNow=outMin; //output signal for fuel pump  0-1000
  startMotorTarget=outMin;// output signal for starter motor 0-1000
  startMotorNow=outMin;// output signal for starter motor 0-1000
  gasFlow=false; //gas valve for starting
  fuelFlow=false;//Solenoid Fuel Valve
  glowPower=outMin;//Stop Glow plug if On

  engineMode=modeWaiting;
  startStage=startStage0;
  switchRestartArmed=false;//switch control must be re-cycled (or pot moved) before restarting after an abort
  potMovedAfterAbort=false;//pot must be moved away from idle and back to re-arm restart
  
  fuelServo.write(MIN_MICROS);
  startServo.write(MIN_MICROS);
  digitalWrite(GlowPin,LOW);//glow plug relay off
  digitalWrite(GasPin,LOW);
  digitalWrite(Fuel_Solenoid_Pin,LOW);
  gasOnTime=0;
   
}

//Active Valve solenoid pulse: returns the fuel solenoid command for the current
//1 Hz period. The solenoid is held on for (duty*period) ms and off for the rest,
//so the average fuel flow is duty x (minimum pump flow). The solenoid is limited
//to at most one on/off cycle per second (period >= 1000 ms, max frequency 1 Hz).
bool ActiveValveUpdate(float duty)
{
  unsigned long now=millis();
  if ((now-activeValveTimeOld)>=activeValvePeriod)
  {
    activeValveTimeOld=now;
    activeValveOn=(duty>0.0f);
  }
  if ((duty<1.0f)&&((now-activeValveTimeOld)>=(unsigned long)(duty*activeValvePeriod)))
  {
    activeValveOn=false;
  }
  return activeValveOn;
}

void ControlOutput()
{
 //Capture the true commanded fuel flow before the slew/floor clamps below modify it,
 //so Active Valve control can act on the actual desired value.
 int valveDesiredFlow=fuelFlowTarget;

 //Check Limits and stop runaway increment or decrement condition
 if (fuelFlowTarget>outMin)
 {
  if (fuelFlowTarget>(fuelFlowNow+1)) fuelFlowTarget=fuelFlowNow+1;// never exceed more than One point
  //Anti-runaway down-clamp applies only while the pump is above its minimum:
  //below the minimum the solenoid pulse duty carries the reduction, so the
  //commanded value must be allowed to fall below pumpOnValue (otherwise it
  //pins at pumpOnValue-1 and sub-minimum active-valve flow is unreachable).
  if ((fuelFlowNow>pumpOnValue)&&(fuelFlowTarget<(fuelFlowNow-1))) fuelFlowTarget=fuelFlowNow-1;
 }
 if (startMotorTarget>outMin)
{
  if (startMotorTarget>(startMotorNow+1)) startMotorTarget=startMotorNow+1;
  if (startMotorTarget<(startMotorNow-1)) startMotorTarget=startMotorNow-1;
}

  if (fuelFlowTarget>outMax) fuelFlowTarget=outMax;
  
  if (pumpOnValue>outMin)
  {
  //Pump minimum floor applies to the pump OUTPUT only. The commanded value is
  //left free to fall below pumpOnValue so Active Valve pulsing can deliver the
  //lower average flow; the pump itself stays at its minimum in that regime.
 if (fuelFlowNow<pumpOnValue)  fuelFlowNow=(pumpOnValue-1);//required when system starts up as initially fuelFlowNow will be zero
   }
      else if (fuelFlowTarget<outMin) fuelFlowTarget=outMin;
            
    if (startMotorTarget>outMax) startMotorTarget=outMax;
    if (startMotorTarget<outMin) startMotorTarget=outMin;
    if (glowPower>outMax) glowPower=outMax;
    if (glowPower<outMin) glowPower=outMin;
  
   //Active Valve: when the commanded fuel flow is below the pump minimum setpoint
   //(pumpOnValue) the pump cannot run any slower, so hold the pump at its minimum
   //and pulse the fuel solenoid at up to 1 Hz to deliver a lower average flow.
   if ((!pumpPrime)&&(pumpOnValue>outMin)&&(valveDesiredFlow<pumpOnValue))
   {
     if (valveDesiredFlow>outMin)
     {
       //Start pulsing with the valve open so the transition into the pulsed regime
       //never closes the fuel supply (a stale pulse phase could hold the valve
       //closed for up to one full second otherwise).
       if (!activeValveActive) { activeValveOn=true; activeValveTimeOld=millis(); }
       activeValveActive=true;
       activeValveDuty=float(valveDesiredFlow)/float(pumpOnValue);
       if (activeValveDuty<0.0f) activeValveDuty=0.0f;
       if (activeValveDuty>1.0f) activeValveDuty=1.0f;
     }
     else { activeValveActive=false; activeValveDuty=0.0f; }
   }
   else { activeValveActive=false; activeValveDuty=0.0f; }
  
   //Fuel Flow Solenoid control
   if ((!pumpPrime)&&(pumpOnValue>outMin)&&(fuelFlowTarget>=pumpOnValue)) fuelFlow=true;//>= keeps the solenoid on at exactly pumpOnValue (no dead-zone fuel cut)
  else if ((!pumpPrime)&&(pumpOnValue==outMin)&&(fuelFlowTarget>outMin)) fuelFlow=true;
  else if (activeValveActive) fuelFlow=ActiveValveUpdate(activeValveDuty);
   else fuelFlow=false;
   
 
   if ((!ignitionState)&&(pumpPrime)) //in case ECU is in pump prime mode and no ignition
   {
   
    fuelFlowNow=fuelFlowTarget;
    if ((pumpOnValue>outMin)&&(fuelFlowNow<pumpOnValue)) fuelFlowNow=(pumpOnValue-1);//pump min floor also during prime (target may be below it)
    fuelServo.write(map(fuelFlowNow,outMin,outMax,MIN_MICROS,MAX_MICROS));
     digitalWrite(Fuel_Solenoid_Pin,LOW);//Keep Fuel Solenoid off to stop fuel from going to engine
   }
 //fuel flow will be controlled with different delays for RPM below idle and RPM above idle as the response time is different in both cases
 else if ((ignitionState)&&(RPMAvg<idleRPM))//handling low RPM operation while engine running
  {
      if ((fuelFlowTarget>fuelFlowNow)&&((millis()-fuelFlowTimeOld)>lowAccelDelay)&&(tempGrad<maxTempGrad)&&((exTemp+(3*tempGrad))<maxTemp) )//fuel pump control with acceleration delay and temp gradient control
  {
    fuelFlowNow++;
  fuelServo.write(map(fuelFlowNow,outMin,outMax,MIN_MICROS,MAX_MICROS));//Servo command takes a value of min micros-max micros
  fuelFlowTimeOld=millis();
  }
  if ((fuelFlowTarget<fuelFlowNow)&&((millis()-fuelFlowTimeOld)>lowDecelDelay) )//fuel pump control with deceleration delay
  {
    fuelFlowNow--;
    if ((pumpOnValue>outMin)&&(fuelFlowNow<(pumpOnValue-1))) fuelFlowNow=(pumpOnValue-1);//pump never drops below its minimum (stops the floor waggle in the pulsed regime)
  fuelServo.write(map(fuelFlowNow,outMin,outMax,MIN_MICROS,MAX_MICROS));
  fuelFlowTimeOld=millis();
  }
  }
else if ((ignitionState)&&(RPMAvg>=idleRPM))    //handle fuel flow above Idle RPM 
{
  if ((fuelFlowTarget>fuelFlowNow)&&((millis()-fuelFlowTimeOld)>accelerationDelay)&&(tempGrad<maxTempGrad)&&((exTemp+(3*tempGrad))<maxTemp))//fuel pump control with acceleration delay
  {
    fuelFlowNow++;
  fuelServo.write(map(fuelFlowNow,outMin,outMax,MIN_MICROS,MAX_MICROS));
  fuelFlowTimeOld=millis();
  }
  if ((fuelFlowTarget<fuelFlowNow)&&((millis()-fuelFlowTimeOld)>decelerationDelay) )//fuel pump control with deceleration delay
  {
    fuelFlowNow--;
    if ((pumpOnValue>outMin)&&(fuelFlowNow<(pumpOnValue-1))) fuelFlowNow=(pumpOnValue-1);//pump never drops below its minimum (stops the floor waggle in the pulsed regime)
    fuelServo.write(map(fuelFlowNow,outMin,outMax,MIN_MICROS,MAX_MICROS));//Servo command takes a value of 0-180
  fuelFlowTimeOld=millis();
  }
}
if(startMotorTarget>starterTransition)
{
  if ((millis()-startMotorTimeOld)>starterIncDelay) //start motor control loop after starter
  {
    if (startMotorTarget>startMotorNow) startMotorNow++;
      if (startMotorTarget<startMotorNow) startMotorNow--;
   startServo.write(map(startMotorNow,outMin,outMax,MIN_MICROS,MAX_MICROS));
  startMotorTimeOld=millis();
  }
  
  }
  else
  {
     if (startMotorTarget>startMotorNow) startMotorNow++;
      if (startMotorTarget<startMotorNow) startMotorNow--;
   startServo.write(map(startMotorNow,outMin,outMax,MIN_MICROS,MAX_MICROS));
  startMotorTimeOld=millis();
  }

  digitalWrite(GlowPin,(glowPower>outMin)?HIGH:LOW);//glow plug relay: on whenever glowPower is above minimum 
 if(gasFlow) digitalWrite(GasPin,HIGH);
 else digitalWrite(GasPin,LOW);
 if(fuelFlow) digitalWrite(Fuel_Solenoid_Pin,HIGH);
 else digitalWrite(Fuel_Solenoid_Pin,LOW);
    
}

void BtnAFunc(void)
{
  
 errorCode=0;//reset error code
// if (engineMode==modeWaiting) WebServerFunction();//only webserver while waiting
}
void BtnADoubleClick(void)
{   
  if (!serverStarted)
  {
    startServer=true;
    WiFi.disconnect(false);  // Reconnect the network 
       
  }
  else{
   if((!pumpPrime)&&(engineMode==modeWaiting)) 
   {
    engineMode=modeStarterOnly;  
    Serial.println("Starter mode");
   }
   else if((!pumpPrime)&&(engineMode==modeStarterOnly))
   {
    engineMode=modeFuelPumpOnly;
      Serial.println("Fuel Pump mode");
   }
  
   else if((!pumpPrime)&&(engineMode==modeFuelPumpOnly))
 {
  pumpPrime=true;
  engineMode=modeWaiting;
  Serial.println("Pump Prime");
 }
 else if((pumpPrime)&&(engineMode==modeWaiting))
  {
    pumpPrime=false;
    engineMode=modeWaiting;
    Serial.println("Starter +Waiting");
  }
  }

}

// Get Sensor Readings and return JSON object
String getSensorReadings(){
  DynamicJsonDocument readings(2048);
  if (fsavailable&&writefile) readings["datafilename"] = filename;
  else if (!fsavailable) readings["datafilename"] = "File Write Error";
  else if (!writefile) readings["datafilename"] = "Data recording stopped";
  readings["temperature"] = String(exTemp);
  readings["tempGrad"] = String(tempGrad);
  readings["rpm"] =  String(RPMAvg/1000.0); //Send RPM for gauge by dividing with 1000
  readings["throttle"]=String(rcThrottleSignal);
  readings["modecontrol"]=String(rcModeSignal);
  readings["fuel"]=String(fuelFlowNow);//use String(fuelFlowNow/1.0);  to show in fractions
  readings["starter"]=String(startMotorNow);
  readings["glow"]=String(glowPower);
  readings["batvolt"]=String(voltage);
  char tempbuffer[16];
  char tempbuffer2[16];
 ltoa(fsWriteTime,tempbuffer,10);
readings["sdlooptime"]= tempbuffer; // Left as sdlooptime for frontend compat
ltoa(maxLoopTime,tempbuffer2,10);
readings["looptime"]= tempbuffer2;
  maxLoopTime=0;//reset maxLoopTime
  if (gasFlow) readings["gas"]="On";
  else readings["gas"]="Off";
  if (engineMode==modeWaiting)readings["ecumode"]="Auto Start";
else if (engineMode==modeStarterOnly)  readings["ecumode"]="Trial Starter Cntrl";
else if (engineMode==modeFuelPumpOnly)readings["ecumode"]="Trial Fuel Cntrl";
else if ((engineMode==modeStarting)&&(startStage==startStage0)) readings["ecumode"]="Purge";
else if ((engineMode==modeStarting)&&(startStage==startStage1))  readings["ecumode"]="Ramp Up";
else if (engineMode==modeIdling)  readings["ecumode"]="Idling";
else if (engineMode==modeOperating)  readings["ecumode"]="Operating";
else if (engineMode==modeCooldown)  readings["ecumode"]="Cooldown";

 if (pumpPrime) readings["thrmode"]="Pump";
 else readings["thrmode"]="Starter";
//1 = no Ignition,2=max temp exceeded,4=max RPM exceeded, 8=no RC signal,16=Flameout,32=RPM Sensor Failure , 64= No Fuel Failure,128=Unable to reach Idle RPM with full motor power//use by setting and reading bits
  if(bitRead(errorCode,0))  readings["error"]="No ignition";
else if(bitRead(errorCode,1))  readings["error"]="Temp Exceeded";
else if(bitRead(errorCode,2))  readings["error"]="RPM Exceeded";
else if(bitRead(errorCode,3))  readings["error"]="RC Signal Lost";
else if(bitRead(errorCode,4)) readings["error"]="Flameout";
else if(bitRead(errorCode,5)) readings["error"]="RPM Sensor Failure";
else if(bitRead(errorCode,6)) readings["error"]="Unable to accelerate on Fuel";
else if(bitRead(errorCode,7)) readings["error"]="Unable to reach Idle RPM";
else readings["error"]="No Error";
  
 
  String jsonString;
  serializeJsonPretty(readings,jsonString);
  return jsonString;
}

void WebServerFunction()
{
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm); // Set WiFi RF power output to lowest level
  WiFi.softAP(ssid.c_str(), password.c_str());
  
   delay(200);
    WiFi.softAPConfig(local_IP, local_IP, subnet);
    delay(500);
   Serial.println(ssid.c_str());
      Serial.println(password.c_str());
     Serial.println(WiFi.softAPIP());
        serverStarted=true; //WebServer has been started
       delay(3000);
//send webpage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
// request->send_P(200, "text/html", index_html);
  request->send_P(200, "text/html", index_html, processor);
});
//display data terminal with realtime values
 server.on("/page2", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage2);
});

//display data terminal with realtime values
 server.on("/page3", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage3);
});
//display data terminal with realtime values
 server.on("/page4", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage4);
});
//display data terminal with realtime values
 server.on("/page5", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage5);
});
//display data terminal with realtime values
 server.on("/page6", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage6);
});
//display engine lifetime data
 server.on("/page7", HTTP_GET, [](AsyncWebServerRequest *request){
 request->send_P(200, "text/html", htmlPage7);
});
//display dir and download page
 server.on("/page8", HTTP_GET, [](AsyncWebServerRequest *request){
 
 request->send_P(200, "text/html",htmlPage8, page8processor);
});
//display data terminal with realtime values
 server.on("/ModeNormal", HTTP_GET, [](AsyncWebServerRequest *request){
  AbortAll();
   pumpPrime=false;
   engineMode= modeWaiting;
request->send(200, "text/html", htmlPage5);
});
//Mode Starter control where throttle stick controls starter only
 server.on("/ModeStarter", HTTP_GET, [](AsyncWebServerRequest *request){
  AbortAll();
  engineMode=modeStarterOnly; 
request->send(200, "text/html", htmlPage5);
});
//Mode fuel pump control where throttle stick controls fuel pump during operation
 server.on("/ModeFuel", HTTP_GET, [](AsyncWebServerRequest *request){
  AbortAll();
  engineMode=modeFuelPumpOnly; 
request->send(200, "text/html", htmlPage5);
});
//Set pump prime mode so throttle stick controls the pump
 server.on("/PumpPrime", HTTP_GET, [](AsyncWebServerRequest *request){
 pumpPrime=true;
request->send(200, "text/html", htmlPage5);
});
//Reset Error code 
 server.on("/ResetError", HTTP_GET, [](AsyncWebServerRequest *request){
  errorCode=0;//reset error code
 request->send(200, "text/html", htmlPage5);

});
//Stop recording will stop recording data 
 server.on("/StopRecording", HTTP_GET, [](AsyncWebServerRequest *request){
  writefile=false;
 
 request->send(200, "text/html", htmlPage5);

});
//Start recording will restart recording data on the current file 
 server.on("/StartRecording", HTTP_GET, [](AsyncWebServerRequest *request){
   writefile=true;
   
  request->send(200, "text/html", htmlPage5);
});
//New File will open a new file and start recording data on the new file 
 server.on("/NewFile", HTTP_GET, [](AsyncWebServerRequest *request){
  if(file) file.flush();
  initLittleFS();
  request->send(200, "text/html", htmlPage5);
});
//Delete all files and start from a new data file
 server.on("/DeleteAll", HTTP_GET, [](AsyncWebServerRequest *request){
 LittleFS_deleteAll();
  request->send(200, "text/html", htmlPage5);
});
//RPM Sensor type hall
 server.on("/hall", HTTP_GET, [](AsyncWebServerRequest *request){
 cpr=1;
  preferences.begin("settings",false);
 preferences.putInt("cpr",cpr);
  preferences.end();
  Serial.print("RPM Counts Per Revolution set to...");
  Serial.println(String(cpr));
  request->send(200, "text/html", htmlPage2);
});
//RPM Sensor type optical
 server.on("/optical", HTTP_GET, [](AsyncWebServerRequest *request){
 cpr=2;
  preferences.begin("settings",false);
 preferences.putInt("cpr",cpr);
  preferences.end();
  Serial.print("RPM Counts Per Revolution set to...");
  Serial.println(String(cpr));
  request->send(200, "text/html", htmlPage2);
});
 server.on("/download", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("Downloading file...");
 
    if (request->hasParam("download")) {
     String downloadfilename = request->getParam("download")->value();
     Serial.println(downloadfilename);
     File file2 = LittleFS.open("/"+ssid+"/"+downloadfilename, "r");//adding root path
     unsigned long downloadsize = file2.size();
     
      String contentType = "application/octet-stream";
     
      AsyncWebServerResponse *response = request->beginResponse(contentType, downloadsize, [file2](uint8_t *buffer, size_t maxLen, size_t total) mutable ->  size_t
                                                               { 
       return file2.read(buffer, maxLen); });
    downloadfilename.toUpperCase();
     String fname=downloadfilename;
  response->addHeader( "Content-Disposition","attachment;filename="+fname);
         request->send(response);
         
   }
  
  });
   server.on("/delete", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("Deleting file...");
   if (request->hasParam("delete")) {
     String deletefilename= request->getParam("delete")->value();
     Serial.println(deletefilename);
     LittleFS_file_delete(deletefilename);
    }
    request->send(200, "text/html", htmlPage8);
  });


//display  value of ECU parameters at abort time
 server.on("/usage",HTTP_GET, [](AsyncWebServerRequest *request){
  String usagedata;
 DynamicJsonDocument usagedoc(256);
 usagedoc["startCount"] = String(startCount);
 usagedoc["engineStartCount"] = String(engineStartCount);
 usagedoc["runTime"] = String(engineRunTime);
 serializeJsonPretty(usagedoc,usagedata);
 request->send(200, "text/plane", usagedata);
});
server.on("/resetECU",HTTP_GET, [](AsyncWebServerRequest *request){
 startCount=0;
 preferences.begin("settings",false);
 preferences.putInt("startCount",startCount);
 preferences.end();
  request->send(200, "text/html",htmlPage7);
});
server.on("/resetengcnt",HTTP_GET, [](AsyncWebServerRequest *request){
  
 engineStartCount=0;
 preferences.begin("settings",false);
 preferences.putInt("engineStarts",engineStartCount);
  preferences.end();
 request->send(200, "text/html",htmlPage7);
});

server.on("/resetengtm",HTTP_GET, [](AsyncWebServerRequest *request){
  
 engineRunTime=0;
 preferences.begin("settings",false);
 preferences.putInt("runTime",engineRunTime);
 preferences.end();
 request->send(200, "text/plane",htmlPage7);
});

//display  value of ECU parameters at abort time
 server.on("/abortdata",HTTP_GET, [](AsyncWebServerRequest *request){
  String abortdata;
 DynamicJsonDocument abortdoc(1024);
 abortdoc["abexTemp"] = String(abexTemp);
 abortdoc["abRPMAvg"] = String(abRPMAvg);
 abortdoc["abfuelFlowNow"] = String(abfuelFlowNow);
 abortdoc["abstartMotorNow"] = String(abstartMotorNow);
 abortdoc["abgasFlow"] = String(abgasFlow);
 abortdoc["abfuelFlow"] = String(abfuelFlow);
 abortdoc["abignitionState"] = String(abignitionState);
 abortdoc["abrcModeSignal"] = String(abrcModeSignal);
 abortdoc["abrcThrottleSignal"] = String(abrcThrottleSignal);
 abortdoc["abglowPower"] = String(abglowPower);
 abortdoc["abengineMode"] = String(abengineMode);
 abortdoc["abstartStage"] = String(abstartStage);
 abortdoc["abvoltage"] = String(abvoltage);
 abortdoc["aberrorCode"] = String(aberrorCode);   
        
 serializeJsonPretty(abortdoc,abortdata);
 request->send(200, "text/plane", abortdata);
});
//display current value of engine parameters
 server.on("/param",HTTP_GET, [](AsyncWebServerRequest *request){
  //JSON for sending engine parameters through Webserver
DynamicJsonDocument doc(8192);

doc["maxTemp"] = String(maxTemp);
doc["maxTempGrad"] = String(maxTempGrad);
doc["maxRPM"] = String(maxRPM);
 doc["idleRPM"]=String(idleRPM);
 doc["rpmTolerance"]=String(rpmTolerance);
 doc["glowOnRPM"] = String(glowOnRPM);
 doc["glowOffRPM"] = String(glowOffRPM);
 doc["ignitionRPMHigh"]=String(ignitionRPMHigh);
 doc["ignitionRPMLow"]=String(ignitionRPMLow);
 doc["gasOnRPM"]=String(gasOnRPM);
 doc["gasOffRPM"]=String(gasOffRPM);
 doc["starterOffRPM"]=String(starterOffRPM);
 doc["fuelOnRPM"]=String(fuelOnRPM);
 doc["startingTemp"]=String (startingTemp);
 doc["pumpOnValue"]=String(pumpOnValue);
 doc["voltIn"]=String(voltage);
 
 
  String paramdata;
 serializeJsonPretty(doc,paramdata);
 request->send(200, "text/plane", paramdata);
});
//display current value of engine parameters
 server.on("/parampage2",HTTP_GET, [](AsyncWebServerRequest *request){
  //JSON for sending engine parameters through Webserver
DynamicJsonDocument docpage2(8192);
  

 docpage2["purgeTime"]=String(purgeTime);
 docpage2["purgeThrottle"]=String(purgeThrottle);
 docpage2["starterIncDelay"]=String(starterIncDelay);
 docpage2["starterLimit"]=String(starterLimit);
 docpage2["accelerationDelay"]=String(accelerationDelay);
 docpage2["decelerationDelay"]=String(decelerationDelay);
 docpage2["lowAccelDelay"]=String(lowAccelDelay);
 docpage2["lowDecelDelay"]=String(lowDecelDelay);
 docpage2["ignitionThreshold"]=String(ignitionThreshold);
 docpage2["noIgnitionThreshold"]=String(noIgnitionThreshold);
 docpage2["trialModeFuelOnRPM"]=String (trialModeFuelOnRPM);
 docpage2["trialModeFuelFlow"]=String (trialModeFuelFlow);
 docpage2["cpr"]=String(cpr);
 docpage2["MIN_MICROS"]=String(MIN_MICROS);
 docpage2["MAX_MICROS"]=String(MAX_MICROS);
 docpage2["WiFiSSID"]=String(ssid);
 docpage2["WiFiPWD"]=String(password);
 String paramdatapage2;
 serializeJsonPretty(docpage2,paramdatapage2);
 request->send(200, "text/plane", paramdatapage2);
});

 // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
   String json = getSensorReadings();
  request->send(200, "application/json", json);
  json = String();
  });
  // Request for the latest sensor readings
  server.on("/keepalive", HTTP_GET, [](AsyncWebServerRequest *request){
    keepalivecounter=millis();//update keep alive counter to keep transmitting event messages
    });
      // Send a GET request to <ESP_IP>/update?state=<inputMessage>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/update?state=<inputMessage>
    if (request->hasParam(PARAM_ServerState)) {
      inputMessage = request->getParam(PARAM_ServerState)->value();
      inputParam = PARAM_ServerState;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
   
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
     if(inputMessage=="0")
    {
      Serial.println("stopping server");
      serverStarted=false;
      startServer=false;//donot start WiFi server in main loop
    dnsServer.stop();
    server.end();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    }
  });

 // Send a GET request to <ESP_IP>/state
  server.on("/state", HTTP_GET, [] (AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "true");
  });
//server.onNotFound(notFound);  
server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
   
    // GET input values
   
    if (request->hasParam(PARAM_maxTemp)) {
      inputmaxTemp = request->getParam(PARAM_maxTemp)->value();
      if (inputmaxTemp.length()>0){
          maxTemp=inputmaxTemp.toInt();
          Serial.println("max Temp Updated to "+inputmaxTemp);
     preferences.begin("settings",false);
     preferences.putInt("maxTemp",maxTemp);
     preferences.end();
    }}

    if (request->hasParam(PARAM_maxRPM)) {
      inputmaxRPM = request->getParam(PARAM_maxRPM)->value();
      if (inputmaxRPM.length()>0){
          maxRPM=inputmaxRPM.toInt();
          Serial.println("max RPM Updated to "+inputmaxRPM);
     preferences.begin("settings",false);
     preferences.putInt("maxRPM",maxRPM);
     preferences.end();
    }}
     if (request->hasParam(PARAM_maxTempGrad)) {
      inputmaxTempGrad = request->getParam(PARAM_maxTempGrad)->value();
      if (inputmaxTempGrad.length()>0){
          maxTempGrad=inputmaxTempGrad.toInt();
          Serial.println("max Temp Grad Updated to "+inputmaxTempGrad);
     preferences.begin("settings",false);
     preferences.putInt("maxTempGrad",maxTempGrad);
     preferences.end();
    }}
    if (request->hasParam(PARAM_idleRPM)) {
      inputidleRPM = request->getParam(PARAM_idleRPM)->value();
      if (inputidleRPM.length()>0){
          idleRPM=inputidleRPM.toInt();
          Serial.println("idle RPM Updated to "+inputidleRPM);
     preferences.begin("settings",false);
     preferences.putInt("idleRPM",idleRPM);
     preferences.end();
    }}
     if (request->hasParam(PARAM_rpmTolerance)) {
      inputrpmTolerance = request->getParam(PARAM_rpmTolerance)->value();
      if (inputrpmTolerance.length()>0){
          rpmTolerance=inputrpmTolerance.toInt();
          Serial.println("RPM Tolerance Updated to "+inputrpmTolerance);
     preferences.begin("settings",false);
     preferences.putInt("rpmTolerance",rpmTolerance);
     preferences.end();
    }}
     if (request->hasParam(PARAM_glowOnRPM)) {
      inputglowOnRPM = request->getParam(PARAM_glowOnRPM)->value();
      if (inputglowOnRPM.length()>0){
          glowOnRPM=inputglowOnRPM.toInt();
          Serial.println("glow On RPM Updated to "+inputglowOnRPM);
     preferences.begin("settings",false);
     preferences.putInt("glowOnRPM",glowOnRPM);
     preferences.end();
    }}
     if (request->hasParam(PARAM_glowOffRPM)) {
      inputglowOffRPM = request->getParam(PARAM_glowOffRPM)->value();
      if (inputglowOffRPM.length()>0){
          glowOffRPM=inputglowOffRPM.toInt();
          Serial.println("glow Off RPM Updated to "+inputglowOffRPM);
     preferences.begin("settings",false);
     preferences.putInt("glowOffRPM",glowOffRPM);
     preferences.end();
    }}
    if (request->hasParam(PARAM_ignitionRPMHigh)) {
      inputignitionRPMHigh = request->getParam(PARAM_ignitionRPMHigh)->value();
      if (inputignitionRPMHigh.length()>0){
          ignitionRPMHigh=inputignitionRPMHigh.toInt();
          Serial.println("ignition RPM High Updated to "+inputignitionRPMHigh);
     preferences.begin("settings",false);
     preferences.putInt("ignitionRPMHigh",ignitionRPMHigh);
     preferences.end();
    }}
    if (request->hasParam(PARAM_ignitionRPMLow)) {
      inputignitionRPMLow = request->getParam(PARAM_ignitionRPMLow)->value();
      if (inputignitionRPMLow.length()>0){
          ignitionRPMLow=inputignitionRPMLow.toInt();
          Serial.println("ignition RPM Low Updated to "+inputignitionRPMLow);
     preferences.begin("settings",false);
     preferences.putInt("ignitionRPMLow",ignitionRPMLow);
     preferences.end();
    }}
     if (request->hasParam(PARAM_gasOnRPM)) {
      inputgasOnRPM = request->getParam(PARAM_gasOnRPM)->value();
      if (inputgasOnRPM.length()>0){
          gasOnRPM=inputgasOnRPM.toInt();
          Serial.println("Gas On RPM Updated to "+inputgasOnRPM);
     preferences.begin("settings",false);
     preferences.putInt("gasOnRPM",gasOnRPM);
     preferences.end();
    }}
    if (request->hasParam(PARAM_gasOffRPM)) {
      inputgasOffRPM = request->getParam(PARAM_gasOffRPM)->value();
      if (inputgasOffRPM.length()>0){
          gasOffRPM=inputgasOffRPM.toInt();
          Serial.println("Gas Off RPM Updated to "+inputgasOffRPM);
     preferences.begin("settings",false);
     preferences.putInt("gasOffRPM",gasOffRPM);
     preferences.end();
    }}
    if (request->hasParam(PARAM_starterOffRPM)) {
      inputstarterOffRPM = request->getParam(PARAM_starterOffRPM)->value();
      if (inputstarterOffRPM.length()>0){
          starterOffRPM=inputstarterOffRPM.toInt();
          Serial.println("Starter Off RPM Updated to "+inputstarterOffRPM);
     preferences.begin("settings",false);
     preferences.putInt("starterOffRPM",starterOffRPM);
     preferences.end();
    }}
if (request->hasParam(PARAM_fuelOnRPM)) {
      inputfuelOnRPM = request->getParam(PARAM_fuelOnRPM)->value();
      if (inputfuelOnRPM.length()>0){
          fuelOnRPM=inputfuelOnRPM.toInt();
          Serial.println("fuel On RPM Updated to "+inputfuelOnRPM);
     preferences.begin("settings",false);
     preferences.putInt("fuelOnRPM",fuelOnRPM);
     preferences.end();
    }}
    if (request->hasParam(PARAM_pumpOnValue)) {
      inputpumpOnValue = request->getParam(PARAM_pumpOnValue)->value();
      if (inputpumpOnValue.length()>0){
          pumpOnValue=inputpumpOnValue.toInt();
          Serial.println("Pump On Value Updated to "+inputpumpOnValue);
     preferences.begin("settings",false);
     preferences.putInt("pumpOnValue",pumpOnValue);
     preferences.end();
    }}
 if (request->hasParam(PARAM_purgeTime)) {
      inputpurgeTime = request->getParam(PARAM_purgeTime)->value();
      if (inputpurgeTime.length()>0){
          purgeTime=inputpurgeTime.toInt();
          Serial.println("purge time Updated to "+inputpurgeTime);
     preferences.begin("settings",false);
     preferences.putInt("purgeTime",purgeTime);
     preferences.end();
    }}
if (request->hasParam(PARAM_purgeThrottle)) {
      inputpurgeThrottle = request->getParam(PARAM_purgeThrottle)->value();
      if (inputpurgeThrottle.length()>0){
          purgeThrottle=inputpurgeThrottle.toInt();
          Serial.println("purge throttle Updated to "+inputpurgeThrottle);
     preferences.begin("settings",false);
     preferences.putInt("purgeThrottle",purgeThrottle);
     preferences.end();
    }}
 if (request->hasParam(PARAM_starterIncDelay)) {
      inputstarterIncDelay = request->getParam(PARAM_starterIncDelay)->value();
      if (inputstarterIncDelay.length()>0){
          starterIncDelay=inputstarterIncDelay.toInt();
          Serial.println("starter Delay Updated to "+inputstarterIncDelay);
     preferences.begin("settings",false);
     preferences.putInt("starterIDelay",starterIncDelay);
     preferences.end();
    }}
    if (request->hasParam(PARAM_starterLimit)) {
      inputstarterLimit = request->getParam(PARAM_starterLimit)->value();
      if (inputstarterLimit.length()>0){
          starterLimit=inputstarterLimit.toInt();
          Serial.println("starter Limit Updated to "+inputstarterLimit);
     preferences.begin("settings",false);
     preferences.putInt("starterLimit",starterLimit);
     preferences.end();
    }}

 if (request->hasParam(PARAM_accelerationDelay)) {
      inputaccelerationDelay = request->getParam(PARAM_accelerationDelay)->value();
      if (inputaccelerationDelay.length()>0){
          accelerationDelay=inputaccelerationDelay.toInt();
          Serial.println("acceleration Delay Updated to "+inputaccelerationDelay);
     preferences.begin("settings",false);
     preferences.putInt("accelDelay",accelerationDelay);
     preferences.end();
    }}
if (request->hasParam(PARAM_decelerationDelay)) {
      inputdecelerationDelay = request->getParam(PARAM_decelerationDelay)->value();
      if (inputdecelerationDelay.length()>0){
          decelerationDelay=inputdecelerationDelay.toInt();
          Serial.println("deceleration Delay Updated to "+inputdecelerationDelay);
     preferences.begin("settings",false);
     preferences.putInt("decelDelay",decelerationDelay);
     preferences.end();
    }}
 if (request->hasParam(PARAM_lowAccelDelay)) {
      inputlowAccelDelay = request->getParam(PARAM_lowAccelDelay)->value();
      if (inputlowAccelDelay.length()>0){
          lowAccelDelay=inputlowAccelDelay.toInt();
          Serial.println("low RPM Acceleration Delay Updated to "+inputlowAccelDelay);
     preferences.begin("settings",false);
     preferences.putInt("lowADelay",lowAccelDelay);
     preferences.end();
    }}
    if (request->hasParam(PARAM_lowDecelDelay)) {
      inputlowDecelDelay = request->getParam(PARAM_lowDecelDelay)->value();
      if (inputlowDecelDelay.length()>0){
          lowDecelDelay=inputlowDecelDelay.toInt();
          Serial.println("low RPM Deceleration Delay Updated to "+inputlowDecelDelay);
     preferences.begin("settings",false);
     preferences.putInt("lowDDelay",lowDecelDelay);
     preferences.end();
    }}
 if (request->hasParam(PARAM_ignitionThreshold)) {
      inputignitionThreshold = request->getParam(PARAM_ignitionThreshold)->value();
      if (inputignitionThreshold.length()>0){
          ignitionThreshold=inputignitionThreshold.toInt();
          Serial.println("ignition Threshold Updated to "+inputignitionThreshold);
     preferences.begin("settings",false);
     preferences.putInt("ignThreshold",ignitionThreshold);
     preferences.end();
    }}
     if (request->hasParam(PARAM_noIgnitionThreshold)) {
      inputnoIgnitionThreshold = request->getParam(PARAM_noIgnitionThreshold)->value();
      if (inputnoIgnitionThreshold.length()>0){
          noIgnitionThreshold=inputnoIgnitionThreshold.toInt();
          Serial.println("no ignition Threshold Updated to "+inputnoIgnitionThreshold);
     preferences.begin("settings",false);
     preferences.putInt("noIgnThreshold",noIgnitionThreshold);
     preferences.end();
    }}
      if (request->hasParam(PARAM_fuelFlowDetectionTime)) {
      inputfuelFlowDetectionTime = request->getParam(PARAM_fuelFlowDetectionTime)->value();
      if (inputfuelFlowDetectionTime.length()>0){
          fuelFlowDetectionTime=inputfuelFlowDetectionTime.toInt();
          Serial.println("Fuel Flow Detection Time Updated to "+fuelFlowDetectionTime);
     preferences.begin("settings",false);
     preferences.putInt("fuelDetTime",fuelFlowDetectionTime);
     preferences.end();
    }}
     if (request->hasParam(PARAM_fuelFlowDetectionRPM)) {
      inputfuelFlowDetectionRPM = request->getParam(PARAM_fuelFlowDetectionRPM)->value();
      if (inputfuelFlowDetectionRPM.length()>0){
          fuelFlowDetectionRPM=inputfuelFlowDetectionRPM.toInt();
          Serial.println("Fuel Flow Detection RPM Updated to "+fuelFlowDetectionRPM);
     preferences.begin("settings",false);
     preferences.putInt("fuelDetRPM",fuelFlowDetectionRPM);
     preferences.end();
    }}
    if (request->hasParam(PARAM_trialModeFuelOnRPM)) {
      inputtrialModeFuelOnRPM = request->getParam(PARAM_trialModeFuelOnRPM)->value();
      if (inputtrialModeFuelOnRPM.length()>0){
          trialModeFuelOnRPM=inputtrialModeFuelOnRPM.toInt();
          Serial.println("trialModeFuelOnRPM Updated to "+trialModeFuelOnRPM);
     preferences.begin("settings",false);
     preferences.putInt("trModeFuelOnRPM",trialModeFuelOnRPM);
     preferences.end();
    }}
     if (request->hasParam(PARAM_trialModeFuelFlow)) {
      inputtrialModeFuelFlow = request->getParam(PARAM_trialModeFuelFlow)->value();
      if (inputtrialModeFuelFlow.length()>0){
          trialModeFuelFlow=inputtrialModeFuelFlow.toInt();
          Serial.println("Trial Mode Fuel Flow Updated to "+trialModeFuelFlow);
     preferences.begin("settings",false);
     preferences.putInt("trModeFuelFlow",trialModeFuelFlow);
     preferences.end();
    }}
    if (request->hasParam(PARAM_startingTemp)) {
      inputstartingTemp = request->getParam(PARAM_startingTemp)->value();
      if (inputstartingTemp.length()>0){
          startingTemp=inputstartingTemp.toInt();
          Serial.println("starting Temperature Updated to "+startingTemp);
     preferences.begin("settings",false);
     preferences.putInt("startingTemp",startingTemp);
     preferences.end();
    }}
     if (request->hasParam(PARAM_maxMotorVolt)) {
      inputmaxMotorVolt = request->getParam(PARAM_maxMotorVolt)->value();
      if (inputmaxMotorVolt.length()>0){
          maxMotorVolt=inputmaxMotorVolt.toFloat();
          Serial.println("Starter Motor Voltage Updated to "+String(maxMotorVolt));
     preferences.begin("settings",false);
     preferences.putFloat("maxMotorVolt",maxMotorVolt);
     preferences.end();
    }}
    if (request->hasParam(PARAM_maxPumpVolt)) {
      inputmaxPumpVolt = request->getParam(PARAM_maxPumpVolt)->value();
      if (inputmaxPumpVolt.length()>0){
          maxPumpVolt=inputmaxPumpVolt.toFloat();
          Serial.println("Fuel Pump Voltage Updated to "+String(maxPumpVolt));
     preferences.begin("settings",false);
     preferences.putFloat("maxPumpVolt",maxPumpVolt);
     preferences.end();
    }}
 if (request->hasParam(PARAM_MIN_MICROS)) {
      inputMIN_MICROS = request->getParam(PARAM_MIN_MICROS)->value();
      if (inputMIN_MICROS.length()>0){
          MIN_MICROS=inputMIN_MICROS.toInt();
          Serial.println("MIN_MICROS Updated to "+inputMIN_MICROS);
     preferences.begin("settings",false);
     preferences.putInt("MIN_MICROS",MIN_MICROS);
     preferences.end();
    }}

    if (request->hasParam(PARAM_MAX_MICROS)) {
      inputMAX_MICROS = request->getParam(PARAM_MAX_MICROS)->value();
      if (inputMAX_MICROS.length()>0){
          MAX_MICROS=inputMAX_MICROS.toInt();
          Serial.println("MAX_MICROS Updated to "+inputMAX_MICROS);
     preferences.begin("settings",false);
     preferences.putInt("MAX_MICROS",MAX_MICROS);
     preferences.end();
    }}
     if (request->hasParam(PARAM_voltIn)) {
      inputvoltIn = request->getParam(PARAM_voltIn)->value();
      if (inputvoltIn.length()>0){
        float volt=resistorRatio*3.3*(float(voltsensorValue)/4095);
       voltCorrection=inputvoltIn.toFloat()/volt;
          Serial.println("Input voltage calibration factor "+String(voltCorrection));
     preferences.begin("settings",false);
     preferences.putFloat("voltCorr",voltCorrection);
     preferences.end();
    }}

     if (request->hasParam(PARAM_WiFiPWD)) {
      inputWiFiPWD = request->getParam(PARAM_WiFiPWD)->value();
      if (inputWiFiPWD.length()>7){ //WiFi Password has to be more than 7 characters
          password=inputWiFiPWD;
          Serial.println("WiFi Password Updated to "+password);
     preferences.begin("settings",false);
     preferences.putString("PWD",password);
     preferences.end();
    }}

    if (request->hasParam(PARAM_WiFiSSID)) {
      inputWiFiSSID = request->getParam(PARAM_WiFiSSID)->value();
      if (inputWiFiSSID.length()>0){
          ssid=inputWiFiSSID;
          Serial.println("WiFi SSID Updated to "+ssid);
     preferences.begin("settings",false);
     preferences.putString("SSID",ssid);
     preferences.end();
    }}
    
     
    request->send(200, "text/html", "Parameters updated as per input fields<br><a href=\"/\">Return to Home Page</a>");
                                    
  });
dnsServer.start(53, "*", WiFi.softAPIP());
 events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  
  server.addHandler(&events);
   server.begin();  

   }
   
void SaveDataHeader()
{
Data="";
Data+="Time(s)";
Data+=",";
Data+="RPM";
Data+=",";
Data+="RPM Gradient(RPM/s)";
Data+=",";
Data+="Amb Temp(C)";
Data+=",";
Data+="Exhaust Temp(C)";
Data+=",";
Data+="Temp Gradient(C/s)";
Data+=",";
Data+="Throttle";
Data+=",";
Data+="ModeSwitch";
Data+=",";
Data+="Engine Mode";
Data+=",";
Data+="Starter";
Data+=",";
Data+="FuelValve";
Data+=",";
Data+="Fuel";
Data+=",";
Data+="Glow";
Data+=",";
Data+="GAS";
Data+=",";
Data+="Voltage";
Data+=",";
Data+="Error";
Data+=",";
Data+="Loop Time";
Data+=",";
Data+="Flash Write Time";
Data+=",";
Data+="Message";
Data+='\n';
appendFile(LittleFS,filename,Data.c_str());
Data=""; //Empty data string
  }
void SaveData()//Called from Main Loop
{
float x=millis()/1000.0;
 if(( millis()-fsLoopTimeOld)>fsLoopTime)
  {
Data+=x;
Data+=",";
Data+=RPMAvg;
Data+=",";
Data+=rpmGrad;
Data+=",";
Data+=ambTemp;
Data+=",";
Data+=exTemp;
Data+=",";
Data+=tempGrad;
Data+=",";
Data+=rcThrottleSignal;
Data+=",";
Data+=rcModeSignal;
Data+=",";
if (engineMode==modeWaiting )  Data+="Wait";   // 0  //No RPM //AutoStart
else if (engineMode==modeStarting) Data+="Start";// 1  //RPM 0-Idle RPM  //Initiated through mode switch
else if (engineMode==modeIdling) Data+="Idling";//        2  //RPM IdleRPM
else if (engineMode== modeOperating) Data+="Operating";//      3  //RPM IdleRPM-MaxRPM
else if (engineMode==modeCooldown) Data+="Cooling";//     4  //RPM IdleRPM-0
else if (engineMode== modeStarterOnly) Data+="Trial Starter";//   5  //Throttle controls the starter only- used to test the RPM limit starter can go without fuel
else if (engineMode==modeFuelPumpOnly) Data+="Trial Fuel";// 6  //Fuel pump is controlled throu
//Data+=engineMode;
Data+=",";
Data+=startMotorNow;
Data+=",";
if (fuelFlow)Data+="ON";
else Data+="OFF";
Data+=",";
Data+=fuelFlowNow;
Data+=",";
Data+=glowPower;
Data+=",";
if (gasFlow)Data+="ON";
else Data+="OFF";
Data+=",";
Data+=voltage;
Data+=",";
Data+=errorCode;
Data+=",";
Data+=measuredLoopTime;
Data+=",";
Data+=fsWriteTime;
Data+=",";
if (sysMsgPending)//system Message queued by core 1 (AbortAll), formatted here on core 0 only
{
  Data+="Error Code, ";
  Data+=sysMsgErrCode;
  Data+='\n';
  sysMsgPending=false;
}
else Data+='\n';
if((fsavailable)) 
{ 
  if (Data.length()>1024)
{
  fsWriteTime=millis();
  appendFile(LittleFS,filename,Data.substring(0,1024).c_str());
  Data.remove(0,1024);
  fsWriteTime=millis()- fsWriteTime; //calculating time to write to Flash
  if(serverStarted){
  if (fsWriteTime>(fsMaxTime+100))
 { fsMaxCount++;
  if (fsMaxCount>1) 
  {
    fsavailable=false; //if Flash write takes more than fsMaxTime then stop writing
   }
    }
  else fsMaxCount=0;
  }
  
  else if(fsWriteTime>(fsMaxTime)) 
  {
     fsMaxCount++;
    if (fsMaxCount>1) 
    {
      fsavailable=false; //if flash write takes more than limit then stop writing to flash
    }
    
  }
  else fsMaxCount=0;
}
}

fsLoopTimeOld=millis();

  }
     
}
 void initPulseCounter (){                                    // initialise pulse counter
    pcnt_config_t pcntFreqConfig = { };                        // Instance of pulse counter
    pcntFreqConfig.pulse_gpio_num = RPMsensorPin;            // pin assignment for pulse counter 
    pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count Rising edges (=change from high to low logical level) as pulses
    pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
    pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // select ESP32 pulse counter unit 0
    pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig);                         // configur rigisters of the pulse counter
  
    pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pause puls counter unit
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  
    pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle);  // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT_FREQ_UNIT);                          // enable overflow interrupt

    pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
    pcnt_filter_enable(PCNT_FREQ_UNIT);                        // enable counter glitch filter (damping)
  
    pcnt_counter_resume(PCNT_FREQ_UNIT);                       // resume counting on pulse counter unit
  }
  
  void Read_Reset_PCNT() {                                     // function for reading pulse counter (for timer)
    pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bits

    // resetting counter as if example, delet for application in PiedPiperS
    OverflowCounter = 0;                                       // set overflow counter to zero
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void Read_PCNT() {                                           // function for reading pulse counter (for timer)
    pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bits
  }

  // Replaces placeholder with button section in your web page
String processor(const String& var){
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
    String outputStateValue = outputState();
    buttons+= "<h4>WiFi WebServer - State <span id=\"outputState\"></span></h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"output\" " + outputStateValue + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  return String();
}
String outputState(){
  if(serverStarted){
    return "checked";
  }
  else {
       return "";
  }
   return "";
 }
  void SaveUsageData()
 {
  int tempStartCount;
  preferences.begin("settings",false);
  tempStartCount=preferences.getInt("engineStarts",engineStartCount);
  if(engineStartCount>tempStartCount) preferences.putInt("engineStarts",engineStartCount);
   preferences.putInt("runTime",engineRunTime);
   preferences.end();
 }
 
 void UpdateUsageData()
 {
if ((millis()-engineUsageTimeOld)>saveUsageTime)
{
   engineRunTime++;
   engineUsageTimeOld=millis();
   SaveUsageData();
 }

 }
 void initLittleFS()
 {
  int x=0;
 bool result=false;

  do{ result=LittleFS.begin(true); // true = format if mount fails
  x++;}
    while ((!result)&&(x<5));

 if(!result) {
    Serial.println("LittleFS Mount Failed");
    fsavailable=false;
    fsAvailableFlag=false;
  } else 
  {
    Serial.println("LittleFS Mount Success");
    fsavailable=true;
    fsAvailableFlag=true;
  }
  
  // Ensure the directory exists if needed for LittleFS
  if (!LittleFS.exists("/" + ssid)) {
      LittleFS.mkdir("/" + ssid);
  }
  
  if(file) file.close();
 do {
  if(file) file.close();
  fileseq++;
  char seq[4];
  itoa(fileseq,seq,10);
  String fnametemp="/"+ssid+"/Data"+seq+".txt";
  fnametemp.toCharArray(filename,32);
  String settingsfnametemp="/"+ssid+"/Settings"+seq+".txt";
  settingsfnametemp.toCharArray(settingsfilename,32);
 file = LittleFS.open(filename, "r");
  } while(file);
    saveSettings();
    file = LittleFS.open(filename,FILE_APPEND);
    headerPending=true;//SaveDataHeader runs on core 0 (Task1code) to avoid a cross-core String race
  if (file){ 
   fsavailable=true;
   Serial.print("File opened ");
  Serial.println(filename);
 } else 
 {
  Serial.print("Failed to open file");
  fsavailable=false;
 }
 
   }

String page8processor(const String& var){
  if(var == "PAGE8PLACEHOLDER"){
    String webpage="";
    String path="/"+ssid;
     File root = LittleFS.open(path);
     if(!root) 
     {
      webpage+=" Storage not available";
      return webpage;
     }
      if(!root.isDirectory()){
        Serial.println("Root failure");
   webpage+=" Storage not available";
      return webpage;
  }
  webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name</th><th>Size</th></tr>");
  File dirfile = root.openNextFile();
  while(dirfile){
 
    if(!dirfile.isDirectory()){
      webpage += "<tr><td>"+String(dirfile.name())+"</td>";
      int bytes = dirfile.size();
      String fsize = "";
      if (bytes < 1024)                     fsize = String(bytes)+" B";
      else if(bytes < (1024 * 1024))        fsize = String(bytes/1024.0,1)+" KB";
      else if(bytes < (1024 * 1024 * 1024)) fsize = String(bytes/1024.0/1024.0,1)+" MB";
      else                                  fsize = String(bytes/1024.0/1024.0/1024.0,1)+" GB";
      webpage += "<td>"+fsize+"</td>";
      webpage += "<td>";
    webpage += F("<FORM action='/download' method='get'>"); 
        webpage += F("<button type='submit' name='download'"); 
          webpage += F("' value='"); webpage +=String(dirfile.name()); webpage +=F("'>Download</button>");
         webpage += F("</FORM>"); 
      webpage += "</td>";
      webpage += "<td>";
      webpage += F("<FORM action='/delete' method='get'>"); 
      webpage += F("<button type='submit' name='delete'"); 
           webpage += F("' value='"); webpage +=String("/"+ssid+"/"+dirfile.name()); webpage +=F("'>Delete</button>");
         webpage += F("</FORM>"); 
      webpage += "</td>";
      webpage += "</tr>";
    }
    dirfile = root.openNextFile();
  }
   webpage += F("</table>");
  return webpage;
  }
  return String();
}