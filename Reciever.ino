//*************************************************************************************************************
// Reciever
//*************************************************************************************************************
#include "Arduino.h"
#include "images.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include "winch_lora.h"
#include <HX711_ADC.h>
#include <ESP32Servo.h>

// HX711 strain gauge pins and constants
#define LOADCELL_DOUT_PIN 6  
#define LOADCELL_SCK_PIN  5  
#define LOADCELL_CALIBRATION 27000  // calibration for tension on string in kgf  (fron original code 2280.0)
#define TENSION_CONSTANT 0          // Once tension is added, then keep a samll amount, do not go back to zero

// RPM Encoder pins
#define RPM_ENCODER_A 2
#define RPM_ENCODER_B 39

// Controller button pins and constants
#define THROTTLE_SERVO 4
#define BRAKE_SERVO 3
#define ENGINE_ON_PIN 7
#define ENGINE_ON_THREASHOLD 3000
#define ENGINE_OFF_PIN 19
#define ENGINE_START_PIN 48

#define MAX_SERVO_ANGLE 90
#define MIN_SERVO_ANGLE 0

#define ITERM_MAX 12
#define ITERM_MIN -12


// Define strain guage
HX711_ADC scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

// Define Servo and brake
Servo throttle, brake;  

// Define counters
volatile int8_t interruptStartCounter=0;
volatile int8_t interruptRunCounterUp=0;
volatile int8_t interruptRunCounterDown=0;
volatile int8_t interruptUpdate=false;
volatile bool newDataReady;
// Define RPM variables
volatile int8_t aState;
volatile int8_t aLastState;
volatile int8_t spoolCount = 0;
volatile int spoolRPM = 0;

// Define timer
static hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Define PID Variables
double Kp=0.05;   //proportional gain 05
double Ki=0.08;  //integral gain 08
double Kd=0.11;   //derivative gain 11

int8_t setPointDifference = 0;
int8_t lastSetPoint = 0;
int8_t setPoint=0;

double error = 0;
double iTerm = 0 ;
double deltaError = 0;
double scaleInput = 0;
double lastScaleInput = 0;
double throttleServoAngle;

double brakeServoAngle;

int maxSpoolRPM=0;
int8_t tensionConstant=0;

// Define rx data variables
char rxChannel;
char rxLoad;
char rxCode;
char rxCheck;

// other variables
bool starting=false;
bool signalLoss = false;

int8_t txChannel=5;
int8_t txCode=0;

// Functions
void logo();
void updateOled();
void engineRunningCheck();

// Interrupts
IRAM_ATTR void onTimer();
IRAM_ATTR void getRPM();
IRAM_ATTR void dataReadyISR();

// Setup OLED // addr , freq , i2c group , resolution , rst
SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 

//************************************************************************************************************************
//*******************Setup Loop*******************************************************************************************
//************************************************************************************************************************
void setup() {
 
  // Define interrupt for reading the RPM and line out distance
    pinMode (RPM_ENCODER_A,INPUT);
    pinMode (RPM_ENCODER_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_ENCODER_A), getRPM, CHANGE); 

    attachInterrupt(digitalPinToInterrupt(LOADCELL_DOUT_PIN), dataReadyISR, FALLING);

  // Define engine on sensor pin
    pinMode (ENGINE_ON_PIN,INPUT_PULLDOWN);

  // Define  engine start and engine off pins
    pinMode (ENGINE_START_PIN,OUTPUT);
    digitalWrite(ENGINE_START_PIN,LOW);
    pinMode (ENGINE_OFF_PIN,OUTPUT);
    digitalWrite(ENGINE_OFF_PIN,HIGH);


  // Define timer 
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 500000, true);
    timerAlarmEnable(timer);

  // Define and set servos
    throttle.attach(THROTTLE_SERVO);//THROTTLE_SERVO,THR0TTLE_MIN,THR0TTLE_MAX); 
    brake.attach(BRAKE_SERVO);//BRAKE_SERVO,BRAKE_MIN,BRAKE_MAX);
    throttle.setPeriodHertz(330);   
    brake.setPeriodHertz(330);  
    throttle.write(0);  // 0 min throttle (idle)
    brake.write(120);    // 90 max brake 

  // Start LoRa 
    lora_init();
    delay(100);

  // Start and set display
    factory_display.init();
    factory_display.display();
    factory_display.screenRotate(ANGLE_0_DEGREE);
    factory_display.clear();
    factory_display.display();

  // Start Logo screen
    logo();

    // Define, start, and calibrate and tare scale while log is displayed
    scale.begin();
    //scale.start(1000, true);
    scale.setCalFactor(LOADCELL_CALIBRATION); 
    scale.tare();			

  // Clear OLED
    factory_display.clear();
    factory_display.setFont(ArialMT_Plain_16);

}


//*******************Main Loop********************************************************************
//************************************************************************************************
//************************************************************************************************
void loop() {

//Send and Recieve LoRa data
  switch(state)
  {
    case STATE_TX:
      sprintf(txpacket,"%1i%1i",txChannel,txCode);
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      state=LOWPOWER;
      break;
    case STATE_RX:
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess( );
      break;
    default:
      break;
  }

// Recieve data from remote transmitter
  rxChannel=rxpacket[0];
  rxLoad=rxpacket[1];
  rxCode=rxpacket[2];
  rxCheck=rxpacket[3];

// Loss of signal from remote transmitter will lower RPM and stop engine if needed.
  if( rxCheck=='0')
    {
      interruptRunCounterUp=0;
      signalLoss=false;
    }
  if( rxCheck=='1')
    {
      interruptRunCounterDown=0;
      signalLoss=false;
    }
  if(interruptRunCounterUp > 4 || interruptRunCounterDown <-4)
    {
      signalLoss=true;
      rxLoad='0';
      lora_init();
      interruptRunCounterUp=0;
      interruptRunCounterDown=0;
      engineRunningCheck();
    }  
  //  Reset signal loss
  if(rxCode == '3'  && signalLoss==true) signalLoss=false;


// select setPoint and add tension constant
  switch(rxLoad)
  {
    case '0':
      setPoint=tensionConstant;
      maxSpoolRPM=0;
      error=0;
      iTerm=0;
      deltaError=0;
    break;
    case '1':
      setPoint=6;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=100;
    break;
    case '2':
      setPoint=10;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=200;
    break;
    case '3':
      setPoint=30;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=300;
    break;
    case '4':
      setPoint=40;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=400;
    break;
    case '5':
      setPoint=50;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=500;
    break;
    case '6':
      setPoint=60;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=600;
    break;
    case '7':
      setPoint=70;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=700;
    break;
    case '8':
      setPoint=80;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=800;
    break;
    case '9':
      setPoint=110;
      tensionConstant=TENSION_CONSTANT;
      maxSpoolRPM=1000;
    break;
    default:
      break;
  }


//**********************************************************************************
//  Control servos based on input from remote transmitter. 
//**********************************************************************************

if (newDataReady)
  {
    scaleInput=-scale.getData();
    newDataReady=0;
  }

if (interruptUpdate) 
  {
      //  keep the setPoint from changing too fast
      setPointDifference = setPoint-lastSetPoint;
      if (setPointDifference > 1)  setPoint=lastSetPoint+2;
      else if(setPointDifference < -1)  setPoint=lastSetPoint-2;
      lastSetPoint = setPoint;

      // Proportional
      error = setPoint - scaleInput; 

      // Integral
      iTerm += Ki*error; 
      if (iTerm >=ITERM_MAX) iTerm = ITERM_MAX;
      else if (iTerm <= ITERM_MIN) iTerm = ITERM_MIN;

      // Derivative 
      deltaError = scaleInput - lastScaleInput; 

      //PID control compute
      throttleServoAngle = Kp*error + iTerm - Kd*deltaError; 
      if (throttleServoAngle >= MAX_SERVO_ANGLE) throttleServoAngle = MAX_SERVO_ANGLE;
      else if (throttleServoAngle <= MIN_SERVO_ANGLE) throttleServoAngle = MIN_SERVO_ANGLE;

      lastScaleInput =scaleInput;

      interruptUpdate=false;

      throttle.write(17+throttleServoAngle);

      brake.write(0);  //90 max
      updateOled();
    }
  
 
//**********************************************************************************
//  Starting and checking engne is running.*****************************************
//**********************************************************************************

    if(rxCode == '1'  && starting==false){  
     
      if(digitalRead(ENGINE_ON_PIN) == HIGH){  // check if need to start
        digitalWrite(ENGINE_START_PIN,LOW);  // engine is running-do not start   
      }else{
        digitalWrite(ENGINE_OFF_PIN,HIGH);      // turn engine stop off
        digitalWrite(ENGINE_START_PIN,HIGH);  // start engine
        interruptStartCounter=0;              // start counter
      }
        starting=true;
    }

    if(starting==true  && interruptStartCounter > 3){  
        digitalWrite(ENGINE_START_PIN,LOW);     //  turn off starter
        engineRunningCheck();  // check if engine is running
      }


//******************************************************************************      

  //  Turning off the engine
    if(rxCode == '2'){  // turn off engine
      digitalWrite(ENGINE_OFF_PIN,LOW);
      txCode=0;
      starting=false;

    }
  //  Reset starting command
    if(rxCode == '3'){  
      digitalWrite(ENGINE_OFF_PIN,HIGH);  // turn engine stop off
      starting=false;
    }
}

//*******************Functions********************************************************************
//************************************************************************************************
//************************************************************************************************

// Display logo function on startup
void logo(){
  factory_display.clear();
  factory_display.drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
  factory_display.display();
}

// update values on OLED screen function
void updateOled(){
  factory_display.clear();

  factory_display.drawString(0, 0, "Set");
  factory_display.drawString(62, 0,  String(setPoint));
  factory_display.drawString(104, 0, "kg");

  factory_display.drawString(0, 20, "Error");
  factory_display.drawString(62, 20, String((error)));
  factory_display.drawString(105, 20, "kg");

  factory_display.drawString(0, 40, "RPM");
  factory_display.drawString(62, 40, String(spoolRPM));

  factory_display.display();
}


void engineRunningCheck(){
    if (digitalRead(ENGINE_ON_PIN) == HIGH){  
      txCode=1; // running
    }else{
      txCode=0; // not running
    }
}

// ***** interupt functions  ******************************************************** 
 
//Get encoder signals to calculate RPM
void IRAM_ATTR getRPM() {

  aState = digitalRead(RPM_ENCODER_A);

  if (aState != aLastState) {

    if (digitalRead(RPM_ENCODER_B) != aState) {
      spoolCount++;
    }
    else {
      spoolCount--;
    }
    aLastState = aState;
  }
}

//timer interrupt
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptStartCounter++;
  interruptRunCounterUp++;
  interruptRunCounterDown--;
  interruptUpdate=true;
  spoolRPM=spoolCount*15;
  spoolCount=0;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//scale interrupt
void dataReadyISR() {
   if (scale.update()) {
    newDataReady = 1;
  }
}







