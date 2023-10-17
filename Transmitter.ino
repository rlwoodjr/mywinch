//*************************************************************************************************************
// Transmitter
//*************************************************************************************************************
#include "Arduino.h"
#include "images.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include "winch_lora.h"

// define constants
#define BATTERY_INPUT 2
#define THROTTLE_BUTTON 1
#define ENGINE_STOP_BUTTON 20
#define REWIND_BUTTON 26
#define BUZZER 3

#define THROTTLE_DELAY_TIME 300
#define REWIND_DELAY_TIME 1000
#define STOP_DELAY_TIME 1500
#define TONE_TIME 500
#define MAX_PULL_FORCE 9
#define BUZZER_FREQUENCY 880

// Functions
void logo();
void updateOled();
void checkButtons();


IRAM_ATTR void onTimer0();
IRAM_ATTR void onTimer1();
IRAM_ATTR void doThrottleButton();
IRAM_ATTR void doEngineStopButton();
IRAM_ATTR void doRewindButton();

static hw_timer_t * timer0 = NULL; // general timer
static hw_timer_t * timer1 = NULL; // buzzer timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int8_t interruptBatteryCounter=0;
volatile int8_t interruptStartCounter=0;
volatile int8_t interruptToneCounter=0;
float battry_volts=0;

int8_t txChannel=5;
int8_t txCode=0;    // 0= initial state, 1= start engine, 2 = stop engine, 3= reset for signal loss
char rxChannel;
char rxCode;        //0= initial state engine not running, 1= engine running
bool txCheck=false;


String OLEDLine1 = "";
String OLEDLine2 = "";
String OLEDLine3 = "";

int8_t pullForce = 0;
int8_t pullForceAdd=0;

long setTrottleDelay=0;
long setStopDelay=0;
long setRewindDelay=0;
bool buzzerOnThrottle=true;
bool buzzerOnStop=true;
bool buzzerOnRewind=true;

bool throttleButton = HIGH;  
bool stopButton = HIGH;
bool rewindButton = HIGH;


int8_t engineOn = 0;
int8_t engineStarting = 0;
long throttleDelayCounter = 0;
long stopDelayCounter = 0;
long rewindDelayCounter = 0;
bool readyToStart=false;

// Setup OLED (addr , freq , i2c group , resolution , rst)
SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 



//*******************Setup Loop*******************************************************************
//************************************************************************************************
//************************************************************************************************

void setup() {

  // Setup buzzer
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER,LOW);
  ledcSetup(0, BUZZER_FREQUENCY, 8);
  ledcWriteTone(0, BUZZER_FREQUENCY);
  ledcAttachPin(BUZZER, 0);

  //  Holding STOP and REWIND buttons will cause the controller to sleep, The THROTTLE
  //  button will turn on the controller
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_1,0); //1 = High, 0 = Low
 
  // Interrupt for buttons (throttle, engine stop, and engine rewind)
  attachInterrupt(digitalPinToInterrupt(THROTTLE_BUTTON), doThrottleButton, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENGINE_STOP_BUTTON), doEngineStopButton, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(REWIND_BUTTON), doRewindButton, CHANGE); 
 
  // configure pins
  pinMode(THROTTLE_BUTTON, INPUT_PULLUP);
  pinMode(ENGINE_STOP_BUTTON, INPUT_PULLUP);
  pinMode(REWIND_BUTTON, INPUT_PULLUP);
  pinMode(BATTERY_INPUT, INPUT_PULLUP);

  // set up a timer 
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, 1000000, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 100000, true);
  timerAlarmEnable(timer1);

  // Setup LoRa (see winch_LoRa.h)
  lora_init();
  delay(100);

  // Setup OLED display
  factory_display.init();
  factory_display.clear();
  factory_display.display();
  factory_display.setFont(ArialMT_Plain_16);

  // Show statup Logo ( see images.h)
  logo();
  delay(300);
  factory_display.clear();
  
}

//*******************Main Loop********************************************************************
//************************************************************************************************
//************************************************************************************************

void loop()
{
  switch(state)
  {
    case STATE_TX:
     sprintf(txpacket,"%1i%1i%1i%1i",txChannel,pullForce,txCode,txCheck);
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



  rxChannel=rxpacket[0];
  rxCode=rxpacket[1];

  switch(rxCode)
    {
      case '0': // engine has not started since reciever reset or engine off
        engineOn = 0;
        txCode=0;
      break;
      case '1': // engine started
        engineOn = 1;
        txCode=0; 
      break;
      default:
      break;
  }

  updateOled();
  checkButtons();


   if(readyToStart && interruptStartCounter>4){  //disarm after set time
     txCode=0;
     readyToStart=false;  
   }

  // Turn buzzer off
  if (interruptToneCounter>3){
    ledcWrite(0,0);
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

    if (interruptBatteryCounter>2){
            battry_volts=analogRead(BATTERY_INPUT)*0.0017;
      interruptBatteryCounter=0;
    }


      // Update values for the OLED Display
      OLEDLine1 = "TX " + String(txpacket)+" EO"+String(engineOn);
      OLEDLine2 ="RX " +  String(rxpacket)+"  "+ String(battry_volts);
      OLEDLine3 = "RSSI " + String(Rssi);


      factory_display.clear();
      factory_display.drawString(0, 0, OLEDLine1);
      factory_display.drawString(0, 20, OLEDLine2);
      factory_display.drawString(0, 40, OLEDLine3);
      factory_display.display();
   //   }
}



// Throttle button *****************************************************************
void checkButtons(){

  throttleDelayCounter=(setTrottleDelay -  millis());
  if(throttleButton == HIGH){
    pullForceAdd=1*engineOn;

  }else if(throttleButton == LOW && throttleDelayCounter >0  && pullForce < MAX_PULL_FORCE){
    pullForce+=pullForceAdd;
    pullForceAdd=0;

    if(readyToStart ){
      txCode=1;  
    }
  }
  // Once Throttle button is released, reduce the pull force
  if(throttleButton == HIGH && throttleDelayCounter < 0  && pullForce >0){
    setTrottleDelay = millis() + THROTTLE_DELAY_TIME;    
    pullForce-=1;
    pullForceAdd=0;
  }


// Stop engine button ********************************************************

  stopDelayCounter=(setStopDelay -  millis());
  

  if(stopButton == HIGH){
    buzzerOnStop=true;


  }else if(stopButton == LOW && stopDelayCounter < 0){

    if(buzzerOnStop){
      interruptToneCounter=0;
      ledcWrite(0,125);
      buzzerOnStop=false;
    }

    if(rxCode=='1' ||  rxCode=='2'){
      engineOn = 0;
      txCode=2;
    }
    
    if(stopButton == LOW && rewindButton == LOW && stopDelayCounter < -1000){
      esp_deep_sleep_start();  ///put to sleep if reWind button is pressed
    }

  }


// Rewind button ****************************************************


  rewindDelayCounter=(setRewindDelay -  millis());

  if(rewindButton == HIGH){
    buzzerOnRewind=true;

  }else if(rewindButton == LOW && rewindDelayCounter <0 ){

    if(buzzerOnRewind){
        interruptToneCounter=0;
        ledcWrite(0,125);
        buzzerOnRewind=false;
      }

      readyToStart=true;
      txCode=3;
      interruptStartCounter=0;
  }

}



// ***** interupt functions  ******************************************************** 

void IRAM_ATTR doThrottleButton(){
  throttleButton = digitalRead(THROTTLE_BUTTON);
  setTrottleDelay = millis() + THROTTLE_DELAY_TIME;
}

void IRAM_ATTR doEngineStopButton(){
  stopButton =digitalRead(ENGINE_STOP_BUTTON);
  setStopDelay = millis() + STOP_DELAY_TIME;  
}

void IRAM_ATTR doRewindButton(){
  rewindButton = digitalRead(REWIND_BUTTON);
  setRewindDelay = millis() + REWIND_DELAY_TIME; 
}

//timer interrupt general
void IRAM_ATTR onTimer0() {
  portENTER_CRITICAL_ISR(&timerMux);
  txCheck=!txCheck;
  interruptBatteryCounter++;
  interruptStartCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// timer interupt for Buzzer
void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptToneCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

