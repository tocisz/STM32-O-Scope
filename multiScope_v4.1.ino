/*.
* Author: Vítor Barbosa
* This project was based on STM32-O-Scope (pigScope)
* It's gonna comprise some useful tools:
*  Oscilloscope
*  Inductance Meter
*  Ambient (pressure and temperature) sensor
*  Waveform Generator (planned)
* This code is free to use in any non-commercial project
*/

// I've left out Serial comm features, re-enable them if you need based on previous versions


#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <SPI.h>
#include <Fonts/TomThumb.h> // super small font

#undef AMBIENT
#ifdef AMBIENT
//edit Wire.h to set SDA and SCL constants to the desired pins (using PC14 and PC15)
#include <Wire.h>
#include <Adafruit_BMP085.h>
#endif

#ifdef AMBIENT
Adafruit_BMP085 bmp;
#endif


//#include <stdint.h>

// if you get uint8_t error, go to TouchScreen_STM.cpp and comment #include "pins_arduino.h"
// also, you may try changing   #include <WProgram.h> to  #include <Arduino.h>
#include <TouchScreen_STM.h>

#include "Adafruit_ILI9341_8bit_STM.h" //modified lib for 8-bit parallel displays

#define PORTRAIT 0
#define LANDSCAPE 3

// size on landscape mode
#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// Initialize touchscreen
/* --------- LCD-STM32 wiring:

8 bit parallel interface

Data pins
Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
Pin stm32 |PB15|PB14|PB13|PB12|PB11|PB10|PB9|PB8|

Control pins |RD |WR |RS |CS |RST|
Pin stm32    |PA0|PA1|PA2|PA3|PA8|

*/


#define TOUCH_SCREEN_AVAILABLE

#ifdef TOUCH_SCREEN_AVAILABLE

// These are the lcd pins used for touch (you don't need to define them here, but it will make touch setup more clear )
#define LCD_RD PA0

#define LCD_RS PA2
#define LCD_CS PA3
#define LCD_D0 PB8
#define LCD_D1 PB9

#define YP LCD_RS  // must be an analog pin
#define XM LCD_CS  // must be an analog pin
#define YM LCD_D0   // can be a digital pin
#define XP LCD_D1   // can be a digital pin

#define XMIN 500
#define XMINBLACK 320
#define XMAX 3740
#define YMIN 420
#define YMAX 3640

#define MINPRESSURE 10
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
int16_t touchX,touchY;
int16_t last_touchX, last_touchY;

int diffTouchThreshold = 25;
int doubleTapMinTime = 150;
int doubleTapMaxTime = 800;
long lastTapTime;

boolean gotDoubleTap = false;
boolean gotTouch = false;

short touch_position;
//|----|----|----|
//|  5 | 4  | 6  |
//|----|----|----|
//|  1 | 0  | 2  |
//|----|----|----|
//|  9 | 8  | 10 |
//|----|----|----|


#endif




/* For reference on STM32F103CXXX

variants/generic_stm32f103c/board/board.h:#define BOARD_NR_SPI              2
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_NSS_PIN        PA4
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MOSI_PIN       PA7
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MISO_PIN       PA6
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_SCK_PIN        PA5

variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_NSS_PIN        PB12
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MOSI_PIN       PB15
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MISO_PIN       PB14
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_SCK_PIN        PB13

*/


#define PIN_TEST_WAVE       PB1

// Create the lcd object
Adafruit_ILI9341_8bit_STM TFT = Adafruit_ILI9341_8bit_STM();


// Display colours
#define BEAM1_COLOUR ILI9341_GREEN
#define BEAM2_COLOUR ILI9341_RED
#define GRATICULE_COLOUR 0x07FF
#define BEAM_OFF_COLOUR ILI9341_BLACK
#define CURSOR_COLOUR ILI9341_GREEN

// Analog input
#define ANALOG_MAX_VALUE 4096
#define ANALOG_MAX_VOLTAGE 3.3
const int8_t PIN_CH1_ANALOG = PB0;   // Analog input pin: any of LQFP44 pins (PORT_PIN), 10 (PA0), 11 (PA1), 12 (PA2), 13 (PA3), 14 (PA4), 15 (PA5), 16 (PA6), 17 (PA7), 18 (PB0), 19  (PB1)
float samplingTime = 0;
float displayTime = 0;
float timePerSample =0;
float kSamples_per_second =0;

const int VPP_SAMPLES = 10;
int vmax_raw,vmin_raw;
float vmax,vmin,Vpp,Vrms;

// variables for frequency counter
volatile unsigned long fallCount =0; // variable whose value will be changed by interrupt, must be marked as volatile
volatile unsigned long riseCount =0; // variable whose value will be changed by interrupt, must be marked as volatile

unsigned long lastMeasureTime=0;
const int FREQ_COUNT_PERIOD = 1000000; // 1s in uS
volatile long lastFallTime, lastRiseTime;
volatile long highTime=0;
volatile long lowTime=0;

int freqCounter_deltaTime =0;
float frequency =0;
// Don't use PC15 and PC14,they didn't work for me
const int PIN_CH1_FALL = PA15;
const int PIN_CH1_RISE = PA12;


//variables for the inductance meter
const int PIN_INDUCT_METER_TRIG = PB3;
const int PIN_INDUCT_METER_ECHO = PB4;

const double INDUCTANCE_METER_FIXED_CAPACITANCE = 1.E-6;

// Variables for the beam position
uint16_t signalX;
uint16_t signalY;
uint16_t signalY1;
int16_t xZoomFactor = 1;
// yZoomFactor (percentage)
int16_t yZoomFactor = 100; //Adjusted to get 3.3V wave to fit on screen
int16_t yPosition = 0;

// Startup with sweep hold off or on
boolean triggerHeld = 0;


unsigned long sweepDelayFactor = 1;
unsigned long timeFactor = 333;  //timeFactor in microseconds

// Screen dimensions
int16_t myWidth;
int16_t myHeight;

//Trigger stuff
boolean notTriggered;

// Sensitivity is the necessary change in AD value which will cause the scope to trigger.
// If VAD=3.3 volts, then 1 unit of sensitivity is around 0.8mV but this assumes no external attenuator. Calibration is needed to match this with the magnitude of the input signal.

// Trigger is setup in one of 32 positions
#define TRIGGER_POSITION_STEP ANALOG_MAX_VALUE/32
// Trigger default position (half of full scale)
int32_t triggerValue = 1024; //2048

int16_t retriggerDelay = 0;
int8_t triggerType = 3; //0-both 1-negative 2-positive 3-always triggers //default was 2
String triggerType_str="";

//Array for trigger points
uint16_t triggerPoints[2];


// Serial output of samples - off by default. Toggled from UI/Serial commands.
boolean serialOutput = false;


// Create USB serial port
//USBSerial usb;
#define usb Serial1

// Samples - depends on available RAM 6K is about the limit on an STM32F103C8T6
// Bear in mind that the ILI9341 display is only able to display 240x320 pixels, at any time but we can output far more to the serial port, we effectively only show a window on our samples on the TFT.
# define maxSamples 1024*6 //1024*6
uint32_t startSample = 0; //10
uint32_t endSample = maxSamples;

// Array for the ADC data
//uint16_t dataPoints[maxSamples];
uint32_t dataPoints32[maxSamples / 2];
uint16_t *dataPoints = (uint16_t *)&dataPoints32;

//array for computed data (speedup)
uint16_t dataPlot[320]; //max(width,height) for this display


// End of DMA indication
volatile static bool dma1_ch1_Active;
#define ADC_CR1_FASTINT 0x70000 // Fast interleave mode DUAL MODE bits 19-16

adc_smp_rate adc_sampling_time = ADC_SMPR_1_5;
int current_sampling_rate =0;

/*
// available sampling rates for adc
typedef enum adc_smp_rate {
ADC_SMPR_1_5,               //< 1.5 ADC cycles
ADC_SMPR_7_5,               //< 7.5 ADC cycles
ADC_SMPR_13_5,              //< 13.5 ADC cycles
ADC_SMPR_28_5,              //< 28.5 ADC cycles
ADC_SMPR_41_5,              //< 41.5 ADC cycles
ADC_SMPR_55_5,              //< 55.5 ADC cycles
ADC_SMPR_71_5,              //< 71.5 ADC cycles
ADC_SMPR_239_5,             /**< 239.5 ADC cycles
} adc_smp_rate;
*/



//UI variables
short trunkHeight =0;
short branch=0;
bool screenChanged = true;
const int  BOXSIZE=50;

boolean scope_enabled =false, ambientSensor_enabled = false;

float abs_f(float value){
  if(value<0) return value*(-1.0);
  else return value;
}

//these calculate touch functions are made for LANDSCAPE mode
uint16_t calculateX(uint16_t x)
{
  return TFT_WIDTH*(x-XMIN)/(XMAX-XMIN);
}
uint16_t calculateY(uint16_t y)
{
  return TFT_HEIGHT-(TFT_HEIGHT*(y-YMIN))/(YMAX-YMIN);
}


adc_smp_rate changeSamplingTime(adc_smp_rate smp_time,boolean increasing){
  switch(int(smp_time)) {
    case ADC_SMPR_1_5:
    if(increasing) smp_time= ADC_SMPR_7_5;
    break;
    case ADC_SMPR_7_5:
    if(increasing) smp_time= ADC_SMPR_13_5;
    else smp_time=ADC_SMPR_1_5;
    break;
    case ADC_SMPR_13_5:
    if(increasing) smp_time= ADC_SMPR_28_5;
    else smp_time=ADC_SMPR_7_5;
    break;
    case ADC_SMPR_28_5:
    if(increasing) smp_time= ADC_SMPR_41_5;
    else smp_time=ADC_SMPR_13_5;
    break;
    case ADC_SMPR_41_5:
    if(increasing) smp_time= ADC_SMPR_55_5;
    else smp_time=ADC_SMPR_28_5;
    break;
    case ADC_SMPR_55_5:
    if(increasing) smp_time= ADC_SMPR_71_5;
    else smp_time=ADC_SMPR_41_5;
    break;
    case ADC_SMPR_71_5:
    if(increasing) smp_time= ADC_SMPR_239_5;
    else smp_time=ADC_SMPR_55_5;
    break;
    case ADC_SMPR_239_5:
    if(!increasing) smp_time= ADC_SMPR_71_5;
    break;

    // ADC_SMPR_239_5 was disabled because once there, we couldn't change it back.

  }
  return smp_time;
}

void increaseSamplingRate(){
  adc_sampling_time = changeSamplingTime(adc_sampling_time,false);
  setADCs();
}


void decreaseSamplingRate(){
  adc_sampling_time = changeSamplingTime(adc_sampling_time,true);
  setADCs();
}

boolean readTouch(){
  // Retrieve a point
  TFT.setRotation(LANDSCAPE); // makes no sense, but without this there are jitter pixes on the middle, with it on the right...
  pinMode(LCD_RD,OUTPUT);
  digitalWrite(LCD_RD, HIGH);
  TSPoint p = ts.getPoint();
  //set control pins to output again
  TFT_CNTRL->regs->CRL = (TFT_CNTRL->regs->CRL & 0xFFFF0000) | 0x00003333;

  gotTouch=false;
  touchX=calculateX(p.x);
  touchY=calculateY(p.y);
  //this is a patched version, originally it was: if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
  if (/*(p.z > 0) &&*/ touchX<TFT_WIDTH-20) {
    gotTouch = true;
    gotDoubleTap=false;
    int deltaTime = millis()-lastTapTime;
    if(deltaTime<=doubleTapMaxTime && deltaTime>doubleTapMinTime) {
      if(abs(last_touchX-touchX) + abs(last_touchY-touchY) <=2*diffTouchThreshold) {
        gotDoubleTap = true;
        //Serial.println("double tap!!");
      }
    }
    lastTapTime = millis();
    last_touchX = touchX;
    last_touchY = touchY;
    //this will show a circle on touched point, useful for debugging
    TFT.fillCircle(touchX, touchY, 3, ILI9341_NAVY);

    /*  Serial.print("touchX = "); Serial.print(touchX);
    Serial.print("\ttouchY = "); Serial.print(touchY); */
    //      usb.print("\tPressure = "); usb.println(p.z);
  }
  //        TFT.setRotation(PORTRAIT);
  if(gotTouch)  return true;
  else return false;
}

void Oscilloscope_touchActions(){
  if(gotTouch){
    switch (touch_position) {
      case 0:
      incEdgeType();
      break;

      case 1:
      decreasetimeFactor();
      break;

      case 2:
      increasetimeFactor();
      break;

      case 4:
      //  if(gotDoubleTap) increaseZoomFactor();
      increaseSamplingRate();
      break;

      case 8:
      //  if(gotDoubleTap) increaseZoomFactor();
      decreaseSamplingRate();
      break;

      case 5:
      increaseZoomFactor();
      break;

      case 9:
      decreaseZoomFactor();
      break;
    }
  }
}

void touchPos() {
  touch_position = 0;

  if(touchX<TFT_WIDTH/3) {
    touch_position |= 1;
  }
  else if(touchX>TFT_WIDTH*2/3) {
    touch_position |= 2;
  }

  if(touchY<TFT_HEIGHT/3) {
    touch_position |= 4;
  }
  else if(touchY>TFT_HEIGHT*2/3) {
    touch_position |= 8;
  }
}

float sampleADC(int numSamples=3){
  int rawSum =0;
  for(int i=0;i<numSamples;i++){
    rawSum+= analogRead(PIN_CH1_ANALOG);
    delay(100);
  }
  float mean = float(rawSum)/float(numSamples);
  return mean;
}


void blockUntilTouch(int timeout = 2500){
  unsigned long time_0 = millis();
  int delta_time =0;
  do{
    readTouch();
    touchPos();
    delay(50);
    delta_time=millis()-time_0;
    if(delta_time>=timeout) break;
  } while (touch_position != 0);
  //  TFT.setRotation(LANDSCAPE);
}

// factors for getting actual voltages from voltage divider iput
const int numProbes =2;
const float probeFactors[numProbes] ={1,2}; // in order: no attenuation, 1/2 attenuation
//const int probeValues_floating[numProbes]={3500,1500};
float probeFactor =0;
bool negativeVoltagesAllowed = false;

void detectProbe(){
  int value_floating = sampleADC();
  pinMode(PIN_CH1_ANALOG,INPUT_PULLUP);
  int value_pullup = sampleADC();
  pinMode(PIN_CH1_ANALOG,INPUT_PULLDOWN);
  int value_pulldown = sampleADC();
  pinMode(PIN_CH1_ANALOG,INPUT_ANALOG);
  /*  TFT.setTextSize(1);
  TFT.print("Raw values:floating,pull-up,pull-down\n");
  TFT.setTextSize(2);
  String str =""; str+=value_floating;str+=", ";str+=value_pullup; str+=", ";str+=value_pulldown;
  TFT.print(str);
  //  TFT.setTextSize(2);
  TFT.print("\n");
  */
  if (value_floating < (ANALOG_MAX_VALUE/2)){ // we're probably dealing with a standard voltage divider probe
  //just two probes for now
  probeFactor = probeFactors[1]; //6v probe
}
else{
  probeFactor = probeFactors[0]; //no attenuation
}

}

void detectProbeUI(){
  clearTFT();
  TFT.setTextSize(2);                     // Small 26 char / line
  //TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
  //        TFT.setRotation(LANDSCAPE);

  int YStep = 20;
  int YStart = 20;
  TFT.setCursor(0, YStart);
  TFT.print("   Detecting Probe...");
  TFT.print("\n");
  detectProbe();
  TFT.print("OK!");
  TFT.print("\n\n");
  TFT.print("Voltage Range:");
  if(negativeVoltagesAllowed) TFT.print("+-");
  else TFT.print("0-");
  TFT.print(3.3f*probeFactor);
  TFT.print("V");
  blockUntilTouch();
}


void setup()
{


  usb.begin(115200);
  adc_calibrate(ADC1);
  adc_calibrate(ADC2);
  setADCs (); //Setup ADC peripherals for interleaved continuous mode.

  #ifdef AMBIENT
  if (!bmp.begin()) usb.println("bmp sensor init failed, check wiring!");
  #endif

  // The test pulse is a square wave of approx 3.3V (i.e. the STM32 supply voltage) at approx 1 kHz
  // "The Arduino has a fixed PWM frequency of 490Hz" - and it appears that this is also true of the STM32F103 using the current STM32F03 libraries as per
  // STM32, Maple and Maple mini port to IDE 1.5.x - http://forum.arduino.cc/index.php?topic=265904.2520
  // therefore if we want a precise test frequency we can't just use the default uncooked 50% duty cycle PWM output.


  #define  TEST_WAVE_FREQ 1000 //1kHz

  timer_set_period(Timer3, (1000000/TEST_WAVE_FREQ)); //This is in uS. For 1000Hz we have (1s=10^6us) T=10^6us/10^3Hz = 10^3us
  //toggleTestWaveOn();  // moved elsewhere

  // Set up our sensor pin(s)
  pinMode(PIN_CH1_ANALOG, INPUT_ANALOG); // this is the scope wave input
  pinMode(PIN_CH1_FALL, INPUT); // this is tied to scope wave input too(via 2.2k resistor), but is used to count frequency
  pinMode(PIN_CH1_RISE, INPUT); // this is tied to scope wave input too(via 2.2k resistor), but is used to count frequency

  pinMode(PIN_INDUCT_METER_TRIG, OUTPUT);
  pinMode(PIN_INDUCT_METER_ECHO, INPUT);

  TFT.begin();
  // initialize the display
  clearTFT();
  //        TFT.setRotation(PORTRAIT);
  myHeight   = TFT.width();
  myWidth  = TFT.height();
  TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR);

  TFT.setRotation(LANDSCAPE);
  //  TFT.setFont(&TomThumb); this is the smallest font available
  clearTFT();
  //  showGraticule();
  showSplash(); //
  blockUntilTouch(3000);
  clearTFT();
  detectProbeUI();

  notTriggered = true;
  trunkHeight++;

  //interrupt for freq counter was here, moved it to the UI of the latter
}



void Oscilloscope(){
  if ( !triggerHeld  )
  {
    // Wait for trigger
    trigger();
    if ( !notTriggered )
    {
      //    blinkLED();

      // Take our samples
      takeSamples();

      //Blank  out previous plot
      TFTSamplesClear(BEAM_OFF_COLOUR);

      // Show the showGraticule
      showGraticule();

      //Display the samples
      TFTSamples(BEAM1_COLOUR);
      displayTime = (micros() - displayTime);

      //getVpp();
      getVoltages();

      //freqCounter(); //now using timer

      // Display the Labels ( uS/Div, Volts/Div etc).
      showLabels();
      displayTime = micros();

    }else {
      showGraticule();
    }

  }

  // Wait before allowing a re-trigger
  delay(retriggerDelay);
  // DEBUG: increment the sweepDelayFactor slowly to show the effect.
  // sweepDelayFactor ++;
}



void choiceScreen(){
  if(screenChanged){ // caled once, in the beginning
    TFT.fillScreen(ILI9341_BLACK);
    //  TFT.setRotation(LANDSCAPE);

    TFT.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
    TFT.fillRect(0, BOXSIZE, BOXSIZE, BOXSIZE, ILI9341_GREEN);
    TFT.fillRect(0, BOXSIZE*2, BOXSIZE, BOXSIZE, ILI9341_BLUE);
    //  TFT.fillRect(0, BOXSIZE*3, BOXSIZE, BOXSIZE, ILI9341_CYAN);
    //  TFT.fillRect(0, BOXSIZE*4, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
    //  TFT.fillRect(0, BOXSIZE*5, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);
    ambientSensor_enabled=false;
    scope_enabled=false;
    screenChanged = false;
  }

  //TFT.setRotation(LANDSCAPE);
  TFT.setTextSize(3);
  TFT.setCursor(BOXSIZE+20, 15);
  TFT.print("Oscilloscope");

  TFT.setCursor(BOXSIZE+20, BOXSIZE+15);
  TFT.print("Freq. Counter");

  TFT.setCursor(BOXSIZE+20, BOXSIZE*2+15);
  TFT.print("Induct. Meter");

  //  TFT.setCursor(BOXSIZE+20, BOXSIZE*3+15);
  //  TFT.print("Not implemented");

  if(gotTouch){
    if(touchY<BOXSIZE){  //choice 0
      screenChanged =true;
      trunkHeight++;
      branch=0;
      TFT.fillScreen(ILI9341_BLACK);

    }
    if(touchY>BOXSIZE&&touchY<BOXSIZE*2){ //choice 1
      screenChanged =true;
      trunkHeight++;
      branch=1;
      TFT.fillScreen(ILI9341_BLACK);

    }
    if(touchY>BOXSIZE*2&&touchY<BOXSIZE*3){ //choice 2
      screenChanged =true;
      trunkHeight++;
      branch=2;
      TFT.fillScreen(ILI9341_BLACK);

    }
    #ifdef AMBIENT
    if(touchY>BOXSIZE*3&&touchY<BOXSIZE*4){ //choice 3
      screenChanged =true;
      trunkHeight++;
      branch=3;
      TFT.fillScreen(ILI9341_BLACK);
    }
    #endif

  }

}

void fallCounterISR(){
  lastFallTime = micros();
  fallCount++;
  highTime = lastFallTime - lastRiseTime;
}

void riseCounterISR(){
  lastRiseTime = micros();
  riseCount++;
  lowTime =lastRiseTime-lastFallTime;
}

void freqCounterISR(){ //called when timer compare matches
  Timer1.pause();
  //frequency = float(1000000)*float(fallCount)/float(FREQ_COUNT_PERIOD);
  frequency = riseCount;
  riseCount=0;
  fallCount=0;
  Timer1.resume();
}

void freqCounterInit(){
  attachInterrupt(PIN_CH1_FALL,fallCounterISR,FALLING); // TODO should work with one pin
  attachInterrupt(PIN_CH1_RISE,riseCounterISR,RISING);
  Timer1.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer1.setPeriod(FREQ_COUNT_PERIOD); // in uS
  Timer1.setCompare1(1);
  Timer1.attachCompare1Interrupt(freqCounterISR);
}




void freqCounterUI(){
  if(screenChanged){ //use this like a setup()
    //freqCounter_enabled = true;
    freqCounterInit();
    toggleTestWaveOn();
    TFT.fillScreen(ILI9341_BLACK);
    //  TFT.setRotation(LANDSCAPE);

    usb.println("freq counter init");

    //TODO: inidicate working range as above 1kHz
    TFT.setTextSize(2);

    TFT.setCursor((TFT_WIDTH/2)-20,10);
    TFT.print("Frequency:");

    //attachInterrupt(PIN_CH1_ANALOG,cyclesCounterISR,RISING);

    screenChanged=false;
  }


  //  TFT.setRotation(LANDSCAPE);
  TFT.setCursor(0,30);
  TFT.print(frequency);
  TFT.print(" Hz");

  TFT.setCursor(0,60);
  TFT.print(FREQ_COUNT_PERIOD);
  TFT.print(" uS");

}

double getInductance(){
  double pulse,frequency,capacitance,inductance;
  capacitance = INDUCTANCE_METER_FIXED_CAPACITANCE;

  digitalWrite(PIN_INDUCT_METER_TRIG,HIGH);
  delay(5);
  //give some time to charge inductor.
  digitalWrite(PIN_INDUCT_METER_TRIG,LOW);
  delayMicroseconds(100);
  //make sure resination is measured

  //pulse in tumeout for smaller inductors could be 5000

  pulse=pulseIn(PIN_INDUCT_METER_ECHO,HIGH,10000);

  //returns 0 if timeout
  if(pulse>0.1){
    //if a timeout did not occur and it took a reading:
    frequency=1.E6/(2*pulse);
    inductance=1./(capacitance*frequency*frequency*4.*3.14159*3.14159);
    inductance*=1E6;  //note that this is the same as saying inductance = inductance*1E6
    inductance*=2.; //for some reason, was returning half the expected value
    return inductance;
  }

}

void inductanceMeterUI(){

  if(screenChanged){ //use this like a setup()
    //inductanceMeter_enabled = true;

    TFT.fillScreen(ILI9341_BLACK);
    //  TFT.setRotation(LANDSCAPE);

    //TODO  inidcate working range: 80- 30000 uH

    TFT.setTextSize(2);
    TFT.setCursor((TFT_WIDTH/2)-20,10);
    TFT.print("Inductance:");

    //attachInterrupt(PIN_CH1_ANALOG,cyclesCounterISR,RISING);

    screenChanged=false;
  }

  double inductance = getInductance();

  //  TFT.setRotation(LANDSCAPE);
  TFT.setCursor(0,30);

  if(inductance<1.e3){
    TFT.print(inductance);
    TFT.print(" uH  ");
  }
  else if(inductance>1.e3 && inductance<1.e6){
    TFT.print(inductance/1.e3);
    TFT.print(" mH  ");
  }
  else if(inductance>1.e6 && inductance<1.e9){
    TFT.print(inductance/1.e6);
    TFT.print(" H  ");
  }

}



#ifdef AMBIENT
void ambientSensorUI(){
  if(screenChanged){
    ambientSensor_enabled = true;
    TFT.fillScreen(ILI9341_BLACK);
    TFT.setRotation(LANDSCAPE);
    TFT.setTextSize(2);
    screenChanged=false;
  }
  TFT.setRotation(LANDSCAPE);
  TFT.setCursor(0,10);
  TFT.print("Temperature:");
  TFT.print(bmp.readTemperature());
  TFT.print("*C");
  TFT.setCursor(0,30);
  TFT.print("Pressure:");
  TFT.print(bmp.readPressure());
  TFT.print(" Pa");
}
#endif

void UI(){
  /*
  0| intro
  1| choose option
  2| ambient_sensor   scope   freq_counter
  */
  if(trunkHeight==1){
    choiceScreen();
    scope_enabled =false;
    ambientSensor_enabled = false;
  }
  else if(trunkHeight==2){
    if(branch==0){ // oscilloscope
      if(screenChanged){
        scope_enabled = true;
        freqCounterInit();
        toggleTestWaveOn();
        showGraticule();
        showLabels();
        screenChanged=false;
      }
      Oscilloscope_touchActions();
      Oscilloscope();

    }
    else if(branch==1){
      freqCounterUI();
    }
    else if(branch==2){
      inductanceMeterUI();
    }
    #ifdef AMBIENT
    else if(branch==0){ //power_meter
      ambientSensorUI();
    }
    #endif
  }

}

void loop()
{
  #ifdef TOUCH_SCREEN_AVAILABLE
  if (readTouch()) {
    touchPos();
  }
  #endif
  UI();
}

void showGraticule()
{
  TFT.drawRect(0, 0, myWidth, myHeight, GRATICULE_COLOUR);
  // Dot grid - ten distinct divisions (9 dots) in both X and Y axis.
  for (uint16_t TicksX = 1; TicksX < 10; TicksX++)
  {
    for (uint16_t TicksY = 1; TicksY < 10; TicksY++)
    {
      TFT.drawPixel(TicksY * (myWidth / 10), TicksX * (myHeight / 10), GRATICULE_COLOUR);

    }
  }

  int edge_interrupt_falling = 3;   // show more dots for the falling interrupt edge, which may be used for frequency counting
  for (int TicksY = 1; TicksY < 20; TicksY++)
  {
    TFT.drawPixel(TicksY * (myWidth / 20),   edge_interrupt_falling * (myHeight / 10), GRATICULE_COLOUR);

  }

  // Horizontal and Vertical centre lines 5 ticks per grid square with a longer tick in line with our dots
  for (uint16_t TicksX = 0; TicksX < myWidth; TicksX += (myHeight / 50))
  {
    if (TicksX % (myWidth / 10) > 0 )
    {
      TFT.drawFastVLine(TicksX,   (myHeight / 2) - 1, 3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastVLine(TicksX,   (myHeight / 2) - 3, 7, GRATICULE_COLOUR);
    }

  }
  for (uint16_t TicksY = 0; TicksY < myHeight; TicksY += (myHeight / 50) )
  {
    if (TicksY % (myHeight / 10) > 0 )
    {
      TFT.drawFastHLine((myWidth / 2) - 1,  TicksY,  3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastHLine((myWidth / 2) - 3,  TicksY,  7, GRATICULE_COLOUR);
    }
  }
}

void setADCs ()
{
  //  const adc_dev *dev = PIN_MAP[PIN_CH1_ANALOG].adc_device;
  int pinMapADCin = PIN_MAP[PIN_CH1_ANALOG].adc_channel;
  adc_set_sample_rate(ADC1, adc_sampling_time); //=0,58uS/sample.  ADC_SMPR_13_5 = 1.08uS - use this one if Rin>10Kohm,
  adc_set_sample_rate(ADC2, adc_sampling_time); // if not may get some sporadic noise. see datasheet.

  //  adc_reg_map *regs = dev->regs;
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR1 |= ADC_CR1_FASTINT; // Interleaved mode
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  ADC2->regs->CR2 |= ADC_CR2_CONT; // ADC 2 continuos
  ADC2->regs->SQR3 = pinMapADCin;
}


// Crude triggering on positive or negative or either change from previous to current sample.
void trigger()
{
  notTriggered = true;
  switch (triggerType) {
    case 1:
    triggerNegative();
    triggerType_str="neg  ";
    break;
    case 2:
    triggerPositive();
    triggerType_str="pos  ";
    break;
    case 3:
    triggerAlways();
    triggerType_str="any  ";
    break;
    default:
    triggerBoth();
    triggerType_str="both ";

    break;
  }
}

void triggerAlways(){

  triggerPoints[0] = analogRead(PIN_CH1_ANALOG);
  notTriggered=false;
  triggerPoints[0] = triggerPoints[1]; //analogRead(PIN_CH1_ANALOG);


}

void triggerBoth()
{
  triggerPoints[0] = analogRead(PIN_CH1_ANALOG);
  while(notTriggered) {
    triggerPoints[1] = analogRead(PIN_CH1_ANALOG);
    if ( ((triggerPoints[1] < triggerValue) && (triggerPoints[0] > triggerValue)) ||
    ((triggerPoints[1] > triggerValue) && (triggerPoints[0] < triggerValue)) ) {
      notTriggered = false;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(PIN_CH1_ANALOG);
  }
}

void triggerPositive() {
  triggerPoints[0] = analogRead(PIN_CH1_ANALOG);
  while(notTriggered) {
    triggerPoints[1] = analogRead(PIN_CH1_ANALOG);
    if ((triggerPoints[1] > triggerValue) && (triggerPoints[0] < triggerValue) ) {
      notTriggered = false;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(PIN_CH1_ANALOG);
  }
}

void triggerNegative() {
  triggerPoints[0] = analogRead(PIN_CH1_ANALOG);
  while(notTriggered) {
    triggerPoints[1] = analogRead(PIN_CH1_ANALOG);
    if ((triggerPoints[1] < triggerValue) && (triggerPoints[0] > triggerValue) ) {
      notTriggered = false;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(PIN_CH1_ANALOG);
  }
}

void incEdgeType() {
  triggerType += 1;
  if (triggerType > 3)
  {
    triggerType = 0;
  }
  /*
  usb.println(triggerPoints[0]);
  usb.println(triggerPoints[1]);
  usb.println(triggerType);
  */
}

void clearTFT()
{
  TFT.fillScreen(BEAM_OFF_COLOUR);          // Blank the display
}



// Grab the samples from the ADC
// Theoretically the ADC can not go any faster than this.
//
// According to specs, when using 72Mhz on the MCU main clock,the fastest ADC capture time is 1.17 uS. As we use 2 ADCs we get double the captures, so .58 uS, which is the times we get with ADC_SMPR_1_5.
// I think we have reached the speed limit of the chip, now all we can do is improve accuracy.
// See; http://stm32duino.com/viewtopic.php?f=19&t=107&p=1202#p1194

void takeSamples ()
{
  // This loop uses dual interleaved mode to get the best performance out of the ADCs
  //

  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS,
    dataPoints32, DMA_SIZE_32BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));// Receive buffer DMA
    dma_set_num_transfers(DMA1, DMA_CH1, maxSamples / 2);
    dma1_ch1_Active = 1;
    //  regs->CR2 |= ADC_CR2_SWSTART; //moved to setADC
    dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
    //adc_calibrate(ADC1);
    //adc_calibrate(ADC2);
    samplingTime = micros();
    while (dma1_ch1_Active);
    samplingTime = (micros() - samplingTime);
    kSamples_per_second = float(maxSamples*1000)/samplingTime;
    timePerSample = float (float(samplingTime) / float(maxSamples));
    dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
    // regs->CR2 &= ~ADC_CR2_CONT;

  }

  void TFTSamplesClear (uint16_t beamColour)
  {
    for (signalX=1; signalX < myWidth - 2; signalX++)
    {
      //use saved data to improve speed
      TFT.drawLine (signalX,   dataPlot[signalX-1], signalX + 1, dataPlot[signalX], beamColour);
    }
  }


  //gets Vpp and Vrms
  void getVoltages(){
    int step = maxSamples/VPP_SAMPLES;
    vmin_raw =ANALOG_MAX_VALUE;
    vmax_raw =0;
    float squared_samples_sum_raw =0;
    for(int i=0; i<VPP_SAMPLES; i++) {

      vmax_raw = max(dataPoints[i*step],vmax_raw);
      vmin_raw = min(dataPoints[i*step],vmin_raw);
      squared_samples_sum_raw += (dataPoints[i*step])*(dataPoints[i*step]);
    }
    float voltage_factor = ANALOG_MAX_VOLTAGE/ANALOG_MAX_VALUE;
    vmin = vmin_raw*voltage_factor*probeFactor;
    vmax = vmax_raw*voltage_factor*probeFactor;
    Vpp = vmax-vmin;

    Vrms=sqrt(squared_samples_sum_raw/VPP_SAMPLES);
    Vrms*=voltage_factor*probeFactor;

  }


  void TFTSamples (uint16_t beamColour)
  {
    //calculate first sample
    signalY =  ((myHeight * dataPoints[0 * ((endSample - startSample) / (myWidth * timeFactor / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition;
    dataPlot[0]=signalY * 99 / 100 + 1;

    for (signalX=1; signalX < myWidth - 2; signalX++)
    {

      // Scale our samples to fit our screen. Most scopes increase this in steps of 5,10,25,50,100 250,500,1000 etc
      // Pick the nearest suitable samples for each of our myWidth screen resolution points
      signalY1 = ((myHeight * dataPoints[(signalX + 1) * ((endSample - startSample) / (myWidth * timeFactor / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition;
      dataPlot[signalX] = signalY1 * 99 / 100 + 1;
      TFT.drawLine (signalX,   dataPlot[signalX-1], signalX + 1, dataPlot[signalX], beamColour);
      signalY = signalY1;

    }


  }


  void showLabels()
  {
    //        TFT.setRotation(LANDSCAPE);
    TFT.setTextSize(1);

    /*
    TFT.setCursor(10, 160);
    TFT.print( bmp.readTemperature());
    TFT.print(" C  ");
    */

    TFT.setCursor(10, 190);
    // TFT.print("Y=");
    //TFT.print((samplingTime * xZoomFactor) / maxSamples);
    //  TFT.print(timePerSample);
    TFT.print(kSamples_per_second);
    TFT.print("kS/s ");
    //  TFT.print(displayTime);
    //TFT.print(float (1000000 / float(displayTime)));
    TFT.print(int(1000000 / displayTime));
    TFT.print("fps ");
    //TFT.print("timeFactor=");
    //TFT.print("x:");
    float timeBase = (1.0/float(timeFactor))*33333.33;
    TFT.print(timeBase);
    TFT.print("us/div");
    //TFT.print(" yzoom=");
    //TFT.print(yZoomFactor);
    //TFT.print(" ypos=");
    //TFT.print(yPosition);
    TFT.print(" trig=");
    TFT.print(triggerType_str);

    /*
    //TFT.print(maxSamples);
    //  TFT.print(" samples ");
    TFT.print(inputFrequency);
    //  TFT.print("-");
    TFT.print(" Hz ");
    //  TFT.setCursor(10, 190);
    TFT.setTextSize(1);
    */
    TFT.setCursor(10, 210);
    TFT.print("0.3");
    TFT.print(" V/Div ");

    TFT.print(Vpp);
    TFT.print("Vpp ");

    TFT.print(vmax);
    TFT.print("Vm ");

    TFT.print(Vrms);
    TFT.print("Vrms ");

    TFT.print(frequency);
    TFT.print(" Hz  ");

    TFT.setCursor(10,10);
    TFT.print("Thigh:");
    TFT.print(highTime);
    TFT.print("uS ");
    TFT.print("Tlow:");
    TFT.print(lowTime);
    TFT.print("uS ");
    TFT.print("Duty:");
    TFT.print(float(highTime)/float(lowTime+highTime));
    TFT.print("   ");





    //        TFT.setRotation(PORTRAIT);
  }


  void decreasetimeFactor() {
    clearTrace();
    /*
    sweepDelayFactor =  sweepDelayFactor / 2 ;
    if (sweepDelayFactor < 1 ) {

    usb.print("timeFactor=");
    sweepDelayFactor = 1;
  }
  */
  if (timeFactor > 133)
  {
    timeFactor -= 50;
  }
  else timeFactor =100;
  /*if(timeFactor=100) timeFactor-=10;
  if(timeFactor<10) timeFactor=10;
  */
  showTrace();
  usb.print("# timeFactor=");
  usb.println(timeFactor);

}

void increasetimeFactor() {
  clearTrace();
  usb.print("# timeFactor=");
  if (timeFactor < 10000)
  {
    if(timeFactor<133) timeFactor=133;
    else timeFactor += 50;
  }
  //sweepDelayFactor = 2 * sweepDelayFactor ;
  showTrace();
  usb.print("# timeFactor=");
  usb.println(timeFactor);
}

void increaseZoomFactor() {
  clearTrace();
  if ( xZoomFactor < 21) {
    xZoomFactor += 1;
  }
  showTrace();
  usb.print("# Zoom=");
  usb.println(xZoomFactor);

}

void decreaseZoomFactor() {
  clearTrace();
  if (xZoomFactor > 1) {
    xZoomFactor -= 1;
  }
  showTrace();
  Serial.print("# Zoom=");
  Serial.println(xZoomFactor);
  //clearTFT();
}

void clearTrace() {
  TFTSamples(BEAM_OFF_COLOUR);
  showGraticule();
}

void showTrace() {
  showLabels();
  TFTSamples(BEAM1_COLOUR);
}

void scrollRight() {
  clearTrace();
  if (startSample < (endSample - 120)) {
    startSample += 100;
  }
  showTrace();
  Serial.print("# startSample=");
  Serial.println(startSample);


}

void scrollLeft() {
  clearTrace();
  if (startSample > (120)) {
    startSample -= 100;
    showTrace();
  }
  Serial.print("# startSample=");
  Serial.println(startSample);

}

void increaseYposition() {

  if (yPosition < myHeight ) {
    clearTrace();
    yPosition++;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}

void decreaseYposition() {

  if (yPosition > -myHeight ) {
    clearTrace();
    yPosition--;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}


void increaseTriggerPosition() {

  if (triggerValue < ANALOG_MAX_VALUE ) {
    triggerValue += TRIGGER_POSITION_STEP; //trigger position step
  }
  Serial.print("# TriggerPosition=");
  Serial.println(triggerValue);
}

void decreaseTriggerPosition() {

  if (triggerValue > 0 ) {
    triggerValue -= TRIGGER_POSITION_STEP; //trigger position step
  }
  Serial.print("# TriggerPosition=");
  Serial.println(triggerValue);
}

void atAt() {
  usb.println("# Hello");
}

void toggleTestWaveOn () {
  pinMode(PIN_TEST_WAVE, OUTPUT);
  analogWrite(PIN_TEST_WAVE, 75);
  usb.println("# Test Pulse On.");
}

void toggleTestWaveOff () {
  pinMode(PIN_TEST_WAVE, INPUT);
  usb.println("# Test Pulse Off.");
}

uint16 timer_set_period(HardwareTimer timer, uint32 microseconds) {
  if (!microseconds) {
    timer.setPrescaleFactor(1);
    timer.setOverflow(1);
    return timer.getOverflow();
  }

  uint32 cycles = microseconds * (72000000 / 1000000); // 72 cycles per microsecond

  uint16 ps = (uint16)((cycles >> 16) + 1);
  timer.setPrescaleFactor(ps);
  timer.setOverflow((cycles / ps) - 1 );
  return timer.getOverflow();
}

/**
* @brief Enable DMA requests
* @param dev ADC device on which to enable DMA requests
*/

void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}


/**
* @brief Disable DMA requests
* @param dev ADC device on which to disable DMA requests
*/

void adc_dma_disable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 0);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}



void showSplash() {
  TFT.setTextSize(2);                     // Small 26 char / line
  //TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
  int YStep = 20;
  int YStart = 20;
  TFT.setCursor(0, YStart);
  TFT.print("     multiScope");
  TFT.setCursor(0, YStart+=2*YStep);
  TFT.print("1.7MS/s max sampling rate");
  TFT.setCursor(0, YStart+=YStep);
  TFT.print("Max MCU Pin Voltage: 3.3V");
  TFT.setCursor(0, YStart+=2*YStep);
  TFT.print("   Scope Touch Actions:");
  TFT.setCursor(0, YStart+=1.5*YStep);
  TFT.print("Left/Right ->timeFactor");
  TFT.setCursor(0, YStart+=YStep);
  TFT.print("Top/Bottom ->Sampling Rate");
  TFT.setCursor(0, YStart+=YStep);
  TFT.print("Center ->Trigger Type");
  TFT.setTextSize(2);
  //        TFT.setRotation(PORTRAIT);
}
