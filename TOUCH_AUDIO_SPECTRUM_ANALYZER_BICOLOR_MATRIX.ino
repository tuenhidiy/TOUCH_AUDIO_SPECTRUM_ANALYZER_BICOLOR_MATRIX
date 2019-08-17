//************************************************************************************************************//
//************* THE 16x32 BICOLOR RG LED MATRIX TOUCH AUDIO SPECTRUM ANALYZER USING B.A.M METHOD *************//
//************************************************************************************************************//
#include <SPI.h>
#include "fix_fft.h"

// REAL PIN ON ARDUINO

#define blank_pin       3   // BLANK PIN - 74HC595
#define latch_pin       2   // LATCH PIN - 74HC595
#define clock_pin       52  // CLOCK PIN - 74HC595
#define data_pin        51  // DATA PIN - 74HC595

#define RowA_Pin        22  // A PIN - 74HC138
#define RowB_Pin        24  // B PIN - 74HC138
#define RowC_Pin        26  // C PIN - 74HC138
#define RowD_Pin        28  // D PIN - 74HC138

#define IR_Select_C     42  // INPUT C - CD4051 (PIN 9)
#define IR_Select_B     44  // INPUT B - CD4051 (PIN 10)
#define IR_Select_A     46  // INPUT A - CD4051 (PIN 11)
#define IR_Enable       48  // INHIBIT - CD4051 (PIN 6)   

//MAPPING TO PORT

#define Blank_Pin_Bit   5 // PORTE bit 5 - PE5
#define Latch_Pin_Bit   4 // PORTE bit 4 - PE4

#define RowA_Pin_Bit    0 // PORTA bit 0 - PA0
#define RowB_Pin_Bit    2 // PORTA bit 2 - PA2
#define RowC_Pin_Bit    4 // PORTA bit 4 - PA4
#define RowD_Pin_Bit    6 // PORTA bit 6 - PA6
#define OE_Pin_Bit      7 // PORTC bit 7 - PC7

//***********************************************FOR ADC PRESCALE SETUP****************************************//
// Optional for high sampling
#define PRESCALE_32     0
// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//************************************************************************************************************//
#define myPI                  3.14159265358979323846
#define myDPI                 1.2732395
#define myDPI2                0.40528473
#define dist(a, b, c, d)      sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))
#ifndef _swap_int16_t
#define _swap_int16_t(a, b)   { int16_t t = a; a = b; b = t; }
#endif
//*************************************************PHOTOTRANSISTORS******************************************//
#define BAUDRATE        9600              // SERIAL BAURATE
#define CALIB           0                 // CALIB = 1 for Calibration
#define NIGHT_TIME      1                 // NIGHT_TIME = 1, DAY_TIME = 0
#define IR_PRINT        0                 // Print working IR phototransistor on Serial Port

volatile int IR_read_data[16];            // Working IR value Array - Normal Operation
volatile int IR_calib_low[16];            // Calibration LOW Array - Cover all phototransistors in 5 seconds
volatile int IR_calib_high[16];           // Calibration HIGH Array - Put all phtotransistors free in 5 seconds
volatile byte IR_counter = 0;             // Tracking IR number
volatile int IR_Read_State[16];           // The IR reading state ((IR_Read_State = HIGH when IR_read_data > ((Calib_High+Calib_Low)/2)
volatile int IR_State[16];                // The IR current state reading from the analog pin
volatile int Last_IR_State[16];           // The IR previous state reading from the analog pin
volatile int Control_State[16];           // The current state of the control state

volatile int Touch_Counter[16];           // Counting how many touch actions

unsigned long lastDebounceTime = 0;       // The last time the touch action was toggled
unsigned long debounceDelay = 5;          // The debounce time; increase if the touch action flickers

unsigned long samplingtime = 0;
unsigned long samplingtime1 = 0;
//*****************************************Fix FFT VARIABLES***********************************************//

#define AUDIORIGHT      8                  // Right A8
//#define AUDIOLEFT       9                  // Left A9

char im[64], data[64];
int data_avgs[16];
int data_average;
int i=0,val;

//*********************************************For ColorWheel Function****************************************//

#define COLOUR_WHEEL_LENGTH 128

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G;
uint8_t VU_R, VU_G;

//************************************************************************************************************//
#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G (256 colours)

const  byte Size_Y = 16;    // Number of LEDS in Y axis (Top to Bottom)
const  byte Size_X = 32;    // Number of LEDs in X axis (Left to Right)

byte red[4][64];
byte green[4][64];

int level=0;
int row=0;
int BAM_Bit, BAM_Counter=0;

//***************************************** An RG Color Template*********************************************//
struct Color
{
  unsigned char red, green;

  Color(int r, int g) : red(r), green(g) {}
  Color() : red(0), green(0) {}
};

const Color redcolor    = Color(0x0F,0x00);
const Color orangecolor = Color(0x0F,0x0F);
const Color yellowcolor = Color(0x0F,0x09);
const Color greencolor  = Color(0x00,0x0F);
const Color clearcolor  = Color(0x00,0x00);

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 15;

pinMode(latch_pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(RowA_Pin, OUTPUT);
pinMode(RowB_Pin, OUTPUT);
pinMode(RowC_Pin, OUTPUT);
pinMode(RowD_Pin, OUTPUT);

pinMode(IR_Enable, OUTPUT);
digitalWrite(IR_Enable, LOW);

pinMode(IR_Select_C, OUTPUT);
pinMode(IR_Select_B, OUTPUT);
pinMode(IR_Select_A, OUTPUT);

Serial.begin(BAUDRATE);
SPI.begin();
interrupts();
fill_colour_wheel();
analogReference(DEFAULT);

IR_calib_high[0] = 1022 ;
IR_calib_high[1] = 1023 ;
IR_calib_high[2] = 1023 ;
IR_calib_high[3] = 1023 ;
IR_calib_high[4] = 1021 ;
IR_calib_high[5] = 1023 ;
IR_calib_high[6] = 1023 ;
IR_calib_high[7] = 1023 ;
IR_calib_high[8] = 1023 ;
IR_calib_high[9] = 1023 ;
IR_calib_high[10] = 1021 ;
IR_calib_high[11] = 1023 ;
IR_calib_high[12] = 1022 ;
IR_calib_high[13] = 1020 ;
IR_calib_high[14] = 1023 ;
IR_calib_high[15] = 1023 ;
IR_calib_low[0] = 47 ;
IR_calib_low[1] = 53 ;
IR_calib_low[2] = 53 ;
IR_calib_low[3] = 60 ;
IR_calib_low[4] = 51 ;
IR_calib_low[5] = 44 ;
IR_calib_low[6] = 52 ;
IR_calib_low[7] = 47 ;
IR_calib_low[8] = 47 ;
IR_calib_low[9] = 44 ;
IR_calib_low[10] = 48 ;
IR_calib_low[11] = 51 ;
IR_calib_low[12] = 49 ;
IR_calib_low[13] = 55 ;
IR_calib_low[14] = 51 ;
IR_calib_low[15] = 44 ;

#if PRESCALE_32
  // Set prescale to 32 for ADC
  // Sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // For Arduino Mega ADC clock is 16 MHz and a conversion takes 13 ADC clocks, prescaler=32 => fs = 38462 (Hz) => fmax = fs/2
  // *** if FFT_N=064, (38462 /2)/(64/2) = 600 hz per bin
  // *** if FFT_N=128, (38462 /2)/(64/2) = 300 hz per bin
  // *** if FFT_N=256, (38462 /2)/(64/2) = 150 hz per bin
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  sbi(ADCSRA,ADPS0) ;
#endif

}

void loop()
{
  //if ( (unsigned long) (micros() - samplingtime1) > 10  )
    //{    
    // Read Analog Input from Microphone
    for (i=0; i < 64; i++)
    {                                     
      val = analogRead(AUDIORIGHT);   
      data[i] = val;                                   
      im[i] = 0;                                                   
    }
    // Perform the FFT on data for log2(64) = 6
    fix_fft(data,im,6,0);
  
    for (i=0; i< 32;i++)
    {                                      
      data[i] = sqrt(data[i] * data[i] + im[i] * im[i]);
    }
       
      //samplingtime1 = micros();
    //}   
  
  // Read Photo-transistors
  
  if ( (unsigned long) (micros() - samplingtime) > 10  )
    {
      for (IR_counter=0; IR_counter< 8; IR_counter++)
        {  
          digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
          digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
          digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
          IR_read_data[0 + IR_counter] = analogRead(A0);
          IR_read_data[8 + IR_counter] = analogRead(A1); 
        }
      samplingtime = micros();
    }
  
  // Counting touch to change the BIN's color of Spectrum Analyzer

  for (byte i=0; i<16; i++)
  {
  // Read the state of the IR phototransistors into a local variable:
  // Comparing with everage of IR_calib_low and IR_calib_high
    if (IR_read_data[i] >= ((IR_calib_high[i] + IR_calib_low[i])/2))
    {
      IR_Read_State[i]=LOW;
    }
    else
    {
      IR_Read_State[i]=HIGH;
    }

    // Check to see if you just press your finger closely to IR phototransistor
    // (i.e. the IR_Read_State went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:
    // If the state changed, due to noise or pressing:
    if (IR_Read_State[i] != Last_IR_State[i])
      {
      // Reset the debouncing timer
      lastDebounceTime = millis();
      }

  if ((millis() - lastDebounceTime) > debounceDelay) 
    {
    // Whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // If the IR_State has changed:
      if (IR_Read_State[i] != IR_State[i]) 
        {
          IR_State[i] = IR_Read_State[i];
          // Only increase touch counter by 1 if the new IR_State is HIGH
          if (IR_State[i] == HIGH)
          {
            Touch_Counter[i]++;
          }
        }
      }
  // Show on the LED MATRIX:
  // Average together
  data_average = data[i*2] + data[i*2+ 1];
  // Get color from colorwheel function base on Touch counter and led columns  
  get_colour(((Touch_Counter[i]+i)*7) % 127 + 8, &VU_R, &VU_G); 
  for (byte yy=0; yy < 16; yy++) 
  {                        
    if ((yy > data_average + 1))
      {        
        LED(2*i, yy, 0, 0);
        LED(2*i+1, yy, 0, 0);
      }
    else
      {
        LED(2*i, yy, VU_R, VU_G);
        LED(2*i+1, yy, VU_R, VU_G);
      }
    }
    // Save the reading. Next time through the loop, it'll be the Last_IR_State:
    Last_IR_State[i] = IR_Read_State[i];
  } 
}

void LED(int X, int Y, int R, int G)
{
  X = constrain(X, 0, Size_X - 1); 
  Y = constrain(Y, 0, Size_Y - 1);
  
  R = constrain(R, 0, (1 << BAM_RESOLUTION) - 1);
  G = constrain(G, 0, (1 << BAM_RESOLUTION) - 1); 

  int WhichByte = 63 - int(Y*4+ X/8);
  int WhichBit = 7 - (X%8);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][WhichByte], WhichBit, bitRead(R, BAM));

    bitWrite(green[BAM][WhichByte], WhichBit, bitRead(G, BAM));
  }
}

void rowScan(byte row)
{
  
  if (row & 0x01) PORTA |= _BV(RowA_Pin_Bit);   //PORTA |= _BV(0)
    else PORTA &= ~_BV(RowA_Pin_Bit);           //PORTA &= ~_BV(0)
  
  if (row & 0x02) PORTA |= _BV(RowB_Pin_Bit);   //PORTA |= _BV(2)
    else PORTA &= ~_BV(RowB_Pin_Bit);           //PORTA &= ~_BV(2)

  if (row & 0x04) PORTA |= _BV(RowC_Pin_Bit);   //PORTA |= _BV(4)
    else PORTA &= ~_BV(RowC_Pin_Bit);           //PORTA &= ~_BV(4)

  if (row & 0x08) PORTA |= _BV(RowD_Pin_Bit);   //PORTA |= _BV(6)
    else PORTA &= ~_BV(RowD_Pin_Bit);           //PORTA &= ~_BV(6)
}

ISR(TIMER1_COMPA_vect){
  
PORTE |= ((1<<Blank_Pin_Bit));                  // Set BLANK PIN high - 74HC595

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      //Red     
        SPI.transfer(red[0][level + 0]);
        SPI.transfer(red[0][level + 1]);
        SPI.transfer(red[0][level + 2]);
        SPI.transfer(red[0][level + 3]);
      //Green        
        SPI.transfer(green[0][level + 0]);
        SPI.transfer(green[0][level + 1]);
        SPI.transfer(green[0][level + 2]);
        SPI.transfer(green[0][level + 3]);
      break;
    case 1:      
      //Red
        SPI.transfer(red[1][level + 0]);
        SPI.transfer(red[1][level + 1]);
        SPI.transfer(red[1][level + 2]);
        SPI.transfer(red[1][level + 3]);                     
      //Green
        SPI.transfer(green[1][level + 0]);
        SPI.transfer(green[1][level + 1]);
        SPI.transfer(green[1][level + 2]);
        SPI.transfer(green[1][level + 3]);       
      break;
    case 2:     
      //Red
        SPI.transfer(red[2][level + 0]);
        SPI.transfer(red[2][level + 1]);
        SPI.transfer(red[2][level + 2]);
        SPI.transfer(red[2][level + 3]);                           
       //Green
        SPI.transfer(green[2][level + 0]);
        SPI.transfer(green[2][level + 1]);
        SPI.transfer(green[2][level + 2]);
        SPI.transfer(green[2][level + 3]);
      break;
    case 3:
      //Red
        SPI.transfer(red[3][level + 0]);
        SPI.transfer(red[3][level + 1]);
        SPI.transfer(red[3][level + 2]);
        SPI.transfer(red[3][level + 3]); 
      //Green
        SPI.transfer(green[3][level + 0]);
        SPI.transfer(green[3][level + 1]);
        SPI.transfer(green[3][level + 2]);
        SPI.transfer(green[3][level + 3]);        
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

PORTE |= 1<<Latch_Pin_Bit;
//delayMicroseconds(2);
PORTE &= ~(1<<Latch_Pin_Bit);
//delayMicroseconds(2); 
PORTE &= ~(1<<Blank_Pin_Bit);       // Set BLANK PIN low - 74HC595

row++;
level = row<<2;
if(row==16)
row=0;
if(level==64)
level=0;
DDRE |= _BV (Blank_Pin_Bit);      // pinMode (blank_pin, OUTPUT);
}

void Calib_High()
{
  for (IR_counter=0; IR_counter< 8; IR_counter++)
  {  
  digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
  digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
  digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
  delay(20);
  // Read phototransistors and store data into calibration HIGH array
  IR_calib_high[0 + IR_counter] = analogRead(A0);
  delay(20);
  IR_calib_high[8 + IR_counter] = analogRead(A1);
  delay(20); 
  }
  // Print on Serial Port
  for (byte i=0; i<16; i++)
  {
    Serial.print("IR_calib_high[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(IR_calib_high[i]);
    Serial.print(" ;");
    Serial.println("");
  }
}

void Calib_Low()
{
  for (IR_counter=0; IR_counter< 8; IR_counter++)
  {  
  digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
  digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
  digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
  delay(20);
  // Read phototransistors and store data into calibration LOW array
  IR_calib_low[0 + IR_counter] = analogRead(A0);
  delay(20);
  IR_calib_low[8 + IR_counter] = analogRead(A1);
  delay(20);  
  }
  // Print on Serial Port
  for (byte i=0; i<16; i++)
  {
    Serial.print("IR_calib_low[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(IR_calib_low[i]);
    Serial.print(" ;");
    Serial.println("");
  }
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 64);
    memset(green, 0, sizeof(green[0][0]) * 4 * 64);
}

void fillTable(byte R, byte G)
{
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<16; y++)
      {
        LED(x, y, R, G);
      }
    }
}
//*******************************************************COLORWHEEL*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G)
{
  if (p >= (COLOUR_WHEEL_LENGTH-1))         // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)
    p -= ((p-= (COLOUR_WHEEL_LENGTH-1)==0) ? COLOUR_WHEEL_LENGTH-3 : COLOUR_WHEEL_LENGTH-1);         // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)

    
  *R = colourR[p];
  *G = colourG[p];
}

void get_next_colour(uint8_t *R, uint8_t *G)
{
  if (++ColPos >= (COLOUR_WHEEL_LENGTH))   // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)
    ColPos -= (COLOUR_WHEEL_LENGTH);      // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)

  *R = colourR[ColPos];
  *G = colourG[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= (COLOUR_WHEEL_LENGTH))   // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)
  {
    colourPos -= (COLOUR_WHEEL_LENGTH);       // Change from "(p >= COLOUR_WHEEL_LENGTH)" to "(p > (COLOUR_WHEEL_LENGTH-3))" to avoid black color transition (dark color)
  }
}
