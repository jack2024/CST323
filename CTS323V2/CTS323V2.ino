/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/**
 * Read ADC data to buffer.
 * - connects to ADC
 * - reads multiple values from channel
 */
//  edit 29/4/64
#include <SPI.h>
#include <Mcp320x.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <inttypes.h> //short forms for various integer types

#include <avr/io.h>
#include <avr/interrupt.h> //file to be included if using interrupts
#include <compat/deprecated.h>

#include <RotaryEncoder.h>
//Beginning of Auto generated function prototypes by Atmel Studio

#define F_OSC 16000000UL                                                     // oscillator-frequency in Hz
#define UART_BAUD_RATE 9600                                                      // Config baud rate 9600
#define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) ((F_OSC)/((UART_BAUD_RATE)*16l)-1)  // baude rate 9600

#define MAXRESISTANT  800

/* define  */
#define WIREIMPEDANCE 0
/* Temp Raw data Mapping */
#define T100 380
#define T95 400
#define T90 420 
#define T85 440
#define T80 460
#define T75 480 
#define T70 500
#define T65 540
#define T60 600
#define T55 630 
#define T50 670
#define T45 720
#define T40 770 

#define OVERHEAT  75  // ประมาณ 90 องศา
#define MAXTEMP   60  // ประมาณ 70 องศา
#define MINTEMP   45  // ประมาณ 50 องศา

#define ROTARYSTEPS 1
#define ROTARYMIN 1
#define ROTARYMAX 32

#define COM  0
#define STAN 1

#define Mode  5
#define Playpausebtn  A10

/* 74HC595 */
#define clockPin   4
#define dataPin    2
#define latchPin   3

#define SPI_CS      40 // dummy CS pin  

// MCP3208 CS Pin
#define IC1_CS      45       
#define IC2_CS      43       
#define IC3_CS      41       
#define IC4_CS      39

// Relay Control
#define RELAY1_1    35 
#define RELAY1_2    37
#define RELAY2_1    31
#define RELAY2_2    33
#define RELAY3_1    27
#define RELAY3_2    29
#define RELAY4_1    23
#define RELAY4_2    25 

#define FAN1        46
#define FAN2        48

#define BUZZER      7



const int Temp_L1 = A3;
const int Temp_L2 = A4;
const int Temp_L3 = A5;
const int Temp_L4 = A6;

const int Temp_R1 = A7;
const int Temp_R2 = A11;
const int Temp_R3 = A12;
const int Temp_R4 = A13;

#define ADC_VREF    5000     // 5V Vref
#define ADC_CLK     1600000  // SPI clock 1.6MHz
#define SPLS        15        // samples
//#define SWSPL_FREQ  10000    // sample rate 10 KHz

     /*   Range 0 - 1.08 Ohm    */  
#define VOLTREAD  0.0012 // 5/4096 = 0.0012 ; 5 = Vref ; 4096 = ADC 12 bit resolution
#define I_CONST   4.166 // 1.25/0.3 = 4.166 R = 0.3 ohm on board devider Vref of LM317 (1.25V)

RotaryEncoder encoder(A8, A9);

String inputString = ""; // a String to hold incoming data
bool rxComplete = false; // whether the string is complete
byte usartinput[20];
byte bytecount = 0;
int timer1_counter;
volatile byte readnum = 1;
volatile byte startmeasure = false;
volatile byte timerovercount;
volatile byte flagreadtemp ;
byte loopcount = 0 , old_loopcount;
byte workmode , play;

byte data1_8, data9_16, data17_24, data25_32;

byte green1_8, green9_16, green17_24, green25_32;
byte red1_8, red9_16, red17_24, red25_32;

unsigned char count =1, old_readnum ;
unsigned char pos;

int rawtemp , temp_L ,temp_R;

byte Temp1 = 0;
byte Temp2 = 0;
byte Temp3 = 0;
byte Temp4 = 0;
byte Temp5 = 0;
byte Temp6 = 0;
byte Temp7 = 0;
byte Temp8 = 0;

MCP3208 adc(ADC_VREF, SPI_CS);

//Module IIC/I2C Interface บางรุ่นอาจจะใช้ 0x27
LiquidCrystal_I2C lcd1(0x27, 20, 4);
LiquidCrystal_I2C lcd2(0x26, 20, 4);
LiquidCrystal_I2C lcd3(0x25, 20, 4);
LiquidCrystal_I2C lcd4(0x24, 20, 4);

// Function Prototype
void beep (void);
void lampdrive(void);
uint16_t readmcp3208(uint8_t ch);
void cleardisplay(void);
unsigned char readrotary(void);
void readmode(void);
void readplaypause(void);
float calculate_display(uint16_t raw ,uint8_t ch);

int meaheatsinktemp(void);

/*
  SerialEvent occurs whenever a new data comes in the hardware Serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent()  {
  digitalWrite(13, !digitalRead(13));
  while (Serial.available()) {
    // get the new byte:
    usartinput[bytecount] = (char)Serial.read();
    bytecount++;
  
    if ((usartinput[0] == 'S')&&(usartinput[bytecount-1] == '\n'))
    {
      rxComplete = true;
        
      if(workmode == COM)
      {
        bytecount =0;
        if(usartinput[1])
        {
          readnum = usartinput[1];
        }
        if(usartinput[1] > 0)
        {
          
          startmeasure = true;
          loopcount =1;
          cleardisplay();
          play = 1 ;
		  lcd1.setCursor(0, 3);lcd1.print("Scanning     :");
        }
        else
        {
          play = 0 ;
          // Off All Relay
          /* digitalWrite(RELAY1_1, HIGH);
          digitalWrite(RELAY1_2, HIGH);
          digitalWrite(RELAY2_1, HIGH);
          digitalWrite(RELAY2_2, HIGH);
          digitalWrite(RELAY3_1, HIGH);
          digitalWrite(RELAY3_2, HIGH);
          digitalWrite(RELAY4_1, HIGH);
          digitalWrite(RELAY4_2, HIGH); */
        }
        beep();
      }
      // timeout counter
      TCNT2 = 255;
      TIMSK2 &= ~(1 << TOIE2);   // disable timer overflow interrupt
      timerovercount =0; //Reset timerovercount     
    }
      // timeout counter
    if(bytecount ==1){
      TCNT2 = 255;   // preload timer 
      TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
    }
       
  }
}


// The Interrupt Service Routine for Pin Change Interrupt 2
// This routine will only be called on any signal change on K0 and K1: exactly where we need to check.
// $0016 PCINT2 PCINT16:23 – PK0:7 Pin Change Interrupt Request 2
ISR(PCINT2_vect) 
{
  if(workmode == STAN)
  {   
    encoder.tick(); // just call tick() to check the state.
  }
}

// Use Check Timeout of USART Rx
ISR(TIMER2_OVF_vect)
{
  if(timerovercount++ > 30)
  {
    timerovercount =0;
    TCNT2 = 255;
    TIMSK2 &= ~(1 << TOIE2); // disable timer overflow interrupt

    for(byte i = 0; i <= bytecount; i++)
    {
        usartinput[i] = 0;
    } 
    bytecount =0;
  }    
}

//  UART1_RX_vect
ISR(UART1_RX_vect)
{
  digitalWrite(13, !digitalRead(13)); // debug
}

void setup()
{
  // configure PIN mode
  noInterrupts();           // disable all interrupts
  // set up timer with prescaler = 1024
  TCCR2B |= ((1 << CS22)|(1 << CS21)|(1 << CS20));
  // initialize counter
  TCNT2 = 255;
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt

  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE2);    // This enables Pin Change Interrupt PCINT16-PCINT23 that covers the Analog input pins or Port K.
  PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);  // This enables the interrupt for PK0 and PK1 of Port K.
  
  //// Timer 1 Overflow interrupt 4 SEC.
  //TCCR1B = (1<<CS12)|(0<<CS11)|(1<<CS10);     // CLK/1024
  //TCNT1H = 0xFF;                // Over flow 50 ms.
  //TCNT1L = 0x77;
  ////cbi(TIFR,TOV1);                             //Clear overflow bit timer1
  //TIFR1 &= ~(1 << TOV1); 
  ////sbi(TIMSK,TOIE1);                           //Enable timer1 overflow interrupt
  //TIMSK1 |= (1 << TOIE1);
  //// write your setup code here, to run once
  
  // Timer 3 Overflow interrupt 50 ms.
  TCCR3B = (1<<CS32)|(0<<CS31)|(1<<CS30);     // CLK/1024
  TCNT3H = 0xFE;                // Over flow 50 ms.
  TCNT3L = 0x77;
  //Clear overflow bit timer1
  TIFR3 &= ~(1 << TOV3);
  //Enable timer1 overflow interrupt
  TIMSK3 |= (1 << TOIE3);
  // write your setup code here, to run once
  
  pinMode(13, OUTPUT); // build in LED
  
  // Switch PIN
  pinMode(Mode, INPUT_PULLUP);
  pinMode(Playpausebtn, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  // 74HC595 PIN
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  digitalWrite(latchPin, HIGH);
  
  pinMode(SPI_CS, OUTPUT);
  pinMode(IC1_CS, OUTPUT); //CS IC1
  pinMode(IC2_CS, OUTPUT); //CS IC2
  pinMode(IC3_CS, OUTPUT); //CS IC3
  pinMode(IC4_CS, OUTPUT); //CS IC4
  digitalWrite(SPI_CS, HIGH);
  digitalWrite(IC1_CS, HIGH);
  digitalWrite(IC2_CS, HIGH);
  digitalWrite(IC3_CS, HIGH);
  digitalWrite(IC4_CS, HIGH);
  
  pinMode(RELAY1_1, OUTPUT); 
  pinMode(RELAY1_2, OUTPUT);
  pinMode(RELAY2_1, OUTPUT); 
  pinMode(RELAY2_2, OUTPUT);
  pinMode(RELAY3_1, OUTPUT); 
  pinMode(RELAY3_2, OUTPUT);
  pinMode(RELAY4_1, OUTPUT); 
  pinMode(RELAY4_2, OUTPUT);
    
  // Off All Relay
  digitalWrite(RELAY1_1, HIGH);
  digitalWrite(RELAY1_2, HIGH);
  digitalWrite(RELAY2_1, HIGH);
  digitalWrite(RELAY2_2, HIGH);
  digitalWrite(RELAY3_1, HIGH);
  digitalWrite(RELAY3_2, HIGH);
  digitalWrite(RELAY4_1, HIGH);
  digitalWrite(RELAY4_2, HIGH);
  
  pinMode(FAN1, OUTPUT); 
  pinMode(FAN2, OUTPUT);
  digitalWrite(FAN1, LOW);
  digitalWrite(FAN2, LOW);
  
  // initialize Serial
  Serial.begin(9600);
  
  //// Set baud rate 9600 at 16 MHz crystal
  //UBRR1H = (uint8_t)(UART_BAUD_CALC(UART_BAUD_RATE,F_OSC)>>8);
  //UBRR1L = (uint8_t)(UART_BAUD_CALC(UART_BAUD_RATE,F_OSC));
  //
  //// Set frame format data  Asynchronous 8 bit data No parity 1 stop bit
   //UCSR1C = 0;
   //UCSR1C |= (1<<UCSZ11)|(1<<UCSZ10);
  //// Enable receiver and transmitter and receiver interrupt
  //UCSR1B = 0;
  //UCSR1B |= (1<<RXEN1)|(1<<RXCIE1);
 
  inputString.reserve(48);
  interrupts(); // enable all interrupts

  lcd1.begin(); lcd1.backlight(); 
  lcd2.begin(); lcd2.backlight(); 
  lcd3.begin(); lcd3.backlight(); 
  lcd4.begin(); lcd4.backlight(); 
  //delay(500);
  
  lcd1.setCursor(0, 0); lcd1.print("ESP Technologies Ltd");
  lcd1.setCursor(0, 1); lcd1.print("NOVA Contact Tester");

  lcd2.setCursor(0, 0); lcd2.print("01:--- 05:--- 09:---");
  lcd2.setCursor(0, 1); lcd2.print("02:--- 06:--- 10:---");
  lcd2.setCursor(0, 2); lcd2.print("03:--- 07:--- 11:---");
  lcd2.setCursor(0, 3); lcd2.print("04:--- 08:--- 12:---");

  lcd3.setCursor(0, 0); lcd3.print("13:--- 17:--- 21:---");
  lcd3.setCursor(0, 1); lcd3.print("14:--- 18:--- 22:---");
  lcd3.setCursor(0, 2); lcd3.print("15:--- 19:--- 23:---");
  lcd3.setCursor(0, 3); lcd3.print("16:--- 20:--- 24:---");
  
  lcd4.setCursor(0, 0); lcd4.print("25:--- 28:--- 31:---");
  lcd4.setCursor(0, 1); lcd4.print("26:--- 29:--- 32:---");
  lcd4.setCursor(0, 2); lcd4.print("27:--- 30:---       ");
  lcd4.setCursor(0, 3); lcd4.print("  unit = milliohm   ");

  if(digitalRead(Mode))
  {
    workmode = STAN;
  }
  else
  {
    workmode = COM;  
  }
 
  // initialize SPI interface for MCP3208
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  lcd1.setCursor(0, 2);lcd1.print("Test LED please wait");
  // %%%%%%%%%% LED Test Lamp %%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for(byte i = 0; i < 2; i++)
  {
    green1_8 =255; 
    green9_16 =255; 
    green17_24 =255; 
    green25_32 =255;
    red1_8 =0; 
    red9_16 =0; 
    red17_24 =0; 
    red25_32 =0;
    
    lampdrive();
    delay(500);
    
    green1_8 =0; 
    green9_16 =0; 
    green17_24 =0; 
    green25_32 =0;
    red1_8 =255; 
    red9_16 =255; 
    red17_24 =255; 
    red25_32 =255;
    
    lampdrive();    
    delay(500);
  }
    
  green1_8 =0; 
  green9_16 =0; 
  green17_24 =0; 
  green25_32 =0;
  red1_8 =0; 
  red9_16 =0; 
  red17_24 =0; 
  red25_32 =0;
  lampdrive();

  lcd1.setCursor(0, 2);lcd1.print("Contact Qty. :      ");
  lcd1.setCursor(0, 3);lcd1.print("Scan         :");
  //lcd1.print(readnum);
  // set default 
  encoder.setPosition(1);
  
  digitalWrite(BUZZER, LOW); // Test BUZZER ON
  digitalWrite(FAN1, LOW); // Test FAN1 ON
  digitalWrite(FAN2, LOW); // Test FAN2 ON
  delay(100); 
  digitalWrite(BUZZER, HIGH); //BUZZER OFF
  delay(500);
  digitalWrite(FAN1, HIGH); // FAN1 OFF
  digitalWrite(FAN2, HIGH); // FAN2 OFF

}
int sensorValue = 0;
void loop()
{
  
  // **** Test Read ADC Temp  ****************//

  if((flagreadtemp)&&(play ==0))
  {
    flagreadtemp = 0; 
    if(meaheatsinktemp())
    {
      // Off All Relay
      digitalWrite(RELAY1_1, HIGH);
      digitalWrite(RELAY1_2, HIGH);
      digitalWrite(RELAY2_1, HIGH);
      digitalWrite(RELAY2_2, HIGH);
      digitalWrite(RELAY3_1, HIGH);
      digitalWrite(RELAY3_2, HIGH);
      digitalWrite(RELAY4_1, HIGH);
      digitalWrite(RELAY4_2, HIGH);
      play = 0;
    } 

  }
  
  // *****************************************//
  
  float ohmraw;
  uint16_t mcpdata =0 , buffreadtemp =0;
  
  if(workmode == STAN)
  {
    readnum = readrotary();
    
    if(readnum != old_readnum) beep();    
    readplaypause();
  }
  // ################# //
  if(readnum < 10) //readnum 0-9
  {
    if(old_readnum > 9)
    {
      lcd1.setCursor(15, 2);
      lcd1.print("   ");  //3 space
    } 
    else if(old_readnum > 99){
      lcd1.setCursor(16, 2);
      lcd1.print(" ");
    }
    lcd1.setCursor(16, 2);
    lcd1.print(readnum);  
  }
  else if(readnum <100)
  {
    if(old_readnum > 99) // readnum 10-99
    {
      lcd1.setCursor(17, 2);
      lcd1.print(" ");
    }
    lcd1.setCursor(10+5, 2);
    lcd1.print(readnum);  
  }
  else // readnum 100 Up
  {
    lcd1.setCursor(15, 2);
    lcd1.print(readnum);
  }
      
  old_readnum = readnum;  
  readmode();
  
  if((startmeasure == true) && (loopcount != 0))
  {
    buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

/* 	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp;

	buffreadtemp = readmcp3208(loopcount);
	mcpdata = mcpdata + buffreadtemp; */
	
/* 	Serial.print("MCP Data : ");
	Serial.println(mcpdata);
	delay(500); */
	
	ohmraw = calculate_display((mcpdata/10) , loopcount);
	
    lampdrive(); 
    if(workmode == COM){ // if com mode send data
      Serial.print(loopcount/10);
      Serial.print(loopcount%10);
      Serial.print(" : ");
      //ohmraw = readmcp3208(loopcount);  
      //lampdrive(); 
      
      if(ohmraw > 1.0 ) // Max 1.179
      {
        Serial.println("-.---");
      }
      else
      {
        Serial.println(ohmraw-0.1 , 3);
      }
    }
    
    
    if(++loopcount > readnum)
    {
		// jj 7/4/21
		play = 0;
		startmeasure = false;
        loopcount = 0;
        //lcd1.setCursor(0, 3);lcd1.print("Scan         :");
		lcd1.setCursor(0, 3);lcd1.print("Scan Complete:");
		
		// Off All Relay
		digitalWrite(RELAY1_1, HIGH);
	    digitalWrite(RELAY1_2, HIGH);
	    digitalWrite(RELAY2_1, HIGH);
	    digitalWrite(RELAY2_2, HIGH);
	    digitalWrite(RELAY3_1, HIGH);
	    digitalWrite(RELAY3_2, HIGH);
	    digitalWrite(RELAY4_1, HIGH);
	    digitalWrite(RELAY4_2, HIGH); 
		//Beep
		digitalWrite(BUZZER, LOW); // 
		delay(300);
		digitalWrite(BUZZER, HIGH);
      /* loopcount = 1;

      // stop scan when complete loop scan
      if(play ==0){
        startmeasure = false;
        loopcount = 0;
        lcd1.setCursor(0, 3);lcd1.print("Scan         :");
      } */
      
    }         
    delay(100); 
  }
  else
  {
   // Off All Relay
   digitalWrite(RELAY1_1, HIGH);
   digitalWrite(RELAY1_2, HIGH);
   digitalWrite(RELAY2_1, HIGH);
   digitalWrite(RELAY2_2, HIGH);
   digitalWrite(RELAY3_1, HIGH);
   digitalWrite(RELAY3_2, HIGH);
   digitalWrite(RELAY4_1, HIGH);
   digitalWrite(RELAY4_2, HIGH); 
  }
    
  if(loopcount < 10) //loopcount 0-9
  {
    if(old_loopcount > 9)
    {
      lcd1.setCursor(15, 3);
      lcd1.print("   ");
    }
    else if(old_loopcount > 99){
      lcd1.setCursor(16, 3);
      lcd1.print(" ");
    }
    lcd1.setCursor(16, 3);
    lcd1.print(loopcount);  
  }
  else if(loopcount <100)
  {
    if(old_loopcount > 99) // loopcount 10-99
    {
      lcd1.setCursor(17, 3);
      lcd1.print(" ");
    }
    lcd1.setCursor(15, 3);
    lcd1.print(loopcount);  
  }
  else // data 100 Up
  {
    lcd1.setCursor(15, 3);
    lcd1.print(loopcount);
  }
  
  old_loopcount = loopcount;
  delay(10); //loop delay
  
  // for loop Test  
 /* if(play == 0)
  {
	static uint16_t testloopcount;
	if(++testloopcount > 500)
	{
		testloopcount =0;
		play = 1;
        loopcount = 1;
        startmeasure = true;
		cleardisplay();
		delay(200);
		beep();
		beep();
	}
  }
  */
  // for loop Test 
  
    // write your main code here, to run repeatedly

}

void beep (void)
{
  digitalWrite(BUZZER, LOW);
  delay(20); 
  digitalWrite(BUZZER, HIGH);
}

void lampdrive(void){
  
  shiftOut(dataPin, clockPin, MSBFIRST, green25_32);
  shiftOut(dataPin, clockPin, MSBFIRST, red25_32); 
  shiftOut(dataPin, clockPin, MSBFIRST, green17_24);
  shiftOut(dataPin, clockPin, MSBFIRST, red17_24); 
  shiftOut(dataPin, clockPin, MSBFIRST, green9_16);
  shiftOut(dataPin, clockPin, MSBFIRST, red9_16);
  shiftOut(dataPin, clockPin, MSBFIRST, green1_8);
  shiftOut(dataPin, clockPin, MSBFIRST, red1_8);
  digitalWrite(latchPin, HIGH);
  delay(1);
  digitalWrite(latchPin, LOW);

}

void cleardisplay(void)
{
  lcd2.setCursor(0, 0); lcd2.print("01:--- 05:--- 09:---");
  lcd2.setCursor(0, 1); lcd2.print("02:--- 06:--- 10:---");
  lcd2.setCursor(0, 2); lcd2.print("03:--- 07:--- 11:---");
  lcd2.setCursor(0, 3); lcd2.print("04:--- 08:--- 12:---");
  lcd3.setCursor(0, 0); lcd3.print("13:--- 17:--- 21:---");
  lcd3.setCursor(0, 1); lcd3.print("14:--- 18:--- 22:---");
  lcd3.setCursor(0, 2); lcd3.print("15:--- 19:--- 23:---");
  lcd3.setCursor(0, 3); lcd3.print("16:--- 20:--- 24:---"); 
  lcd4.setCursor(0, 0); lcd4.print("25:--- 28:--- 31:---");
  lcd4.setCursor(0, 1); lcd4.print("26:--- 29:--- 32:---");
  lcd4.setCursor(0, 2); lcd4.print("27:--- 30:---       ");
  lcd4.setCursor(0, 3); lcd4.print("  unit = milliohm   ");
  
  green1_8 =0; 
  green9_16 =0; 
  green17_24 =0; 
  green25_32 =0;
  red1_8 =0; 
  red9_16 =0; 
  red17_24 =0; 
  red25_32 =0;
  lampdrive();
}

uint16_t readmcp3208(uint8_t ch )
{

  uint16_t tempdata = 0;
  
  // Power Relay Control
  if(ch <=4){                    // channel 1-4
    digitalWrite(RELAY1_1, LOW);
    digitalWrite(RELAY1_2, LOW);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=5) && (ch <=8)){ // channel 5-8
    digitalWrite(RELAY1_1, LOW);
    digitalWrite(RELAY1_2, LOW);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=9) && (ch <=12)){ // channel 9-12
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, LOW);
    digitalWrite(RELAY2_2, LOW);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=13) && (ch <=16)){ // channel 13-16
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, LOW);
    digitalWrite(RELAY2_2, LOW);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=17) && (ch <=20)){ // channel 17-20
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, LOW);
    digitalWrite(RELAY3_2, LOW);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=21) && (ch <=24)){ // channel 21-14
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, LOW);
    digitalWrite(RELAY3_2, LOW);
    digitalWrite(RELAY4_1, HIGH);
    digitalWrite(RELAY4_2, HIGH);   
  }
  else if ((ch >=25) && (ch <=28)){ // channel 25-28
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, LOW);
    digitalWrite(RELAY4_2, LOW);   
  }
  else if ((ch >=29) && (ch <=32)){ // channel 29-32
    digitalWrite(RELAY1_1, HIGH);
    digitalWrite(RELAY1_2, HIGH);
    digitalWrite(RELAY2_1, HIGH);
    digitalWrite(RELAY2_2, HIGH);
    digitalWrite(RELAY3_1, HIGH);
    digitalWrite(RELAY3_2, HIGH);
    digitalWrite(RELAY4_1, LOW);
    digitalWrite(RELAY4_2, LOW);   
  }
  else
  {
    beep();
    delay(200);
    beep();
    delay(200);  
    beep();
    return 7.86;
  }
   
  //MCP3208 CS PIN Control
  if(ch <=8){                    // channel 1-8
    digitalWrite(IC1_CS, LOW);
    digitalWrite(IC2_CS, HIGH);
    digitalWrite(IC3_CS, HIGH);
    digitalWrite(IC4_CS, HIGH);  
  }
  else if ((ch >=9) && (ch <=16)){ // channel 9-16
    digitalWrite(IC1_CS, HIGH);
    digitalWrite(IC2_CS, LOW);
    digitalWrite(IC3_CS, HIGH);
    digitalWrite(IC4_CS, HIGH);    
  }
  else if ((ch >=17) && (ch <=24)){ // channel 17-24
    digitalWrite(IC1_CS, HIGH);
    digitalWrite(IC2_CS, HIGH);
    digitalWrite(IC3_CS, LOW);
    digitalWrite(IC4_CS, HIGH);    
  }
  else if ((ch >=25) && (ch <=32)){ // channel 24-32
    digitalWrite(IC1_CS, HIGH);
    digitalWrite(IC2_CS, HIGH);
    digitalWrite(IC3_CS, HIGH);
    digitalWrite(IC4_CS, LOW);   
  }
  else{
    beep();
    delay(200);
    beep();
    delay(200);  
    beep();
    return 7.86;
  }
  
  //  wait
  delay(10);
  
  tempdata = 0;
  switch(ch)
  {

    case 1:
	tempdata = adc.read(MCP3208::Channel::SINGLE_0);
    break;
	
    case 2:
	tempdata += adc.read(MCP3208::Channel::SINGLE_1);
    break;
	
    case 3:
	tempdata += adc.read(MCP3208::Channel::SINGLE_2);

    break;
    case 4:
	tempdata += adc.read(MCP3208::Channel::SINGLE_3);

    break;
    case 5:
	tempdata = adc.read(MCP3208::Channel::SINGLE_4);

    break;
    case 6:
	tempdata = adc.read(MCP3208::Channel::SINGLE_5);

    break;
    case 7:
	tempdata = adc.read(MCP3208::Channel::SINGLE_6);

    break;
    case 8:
	tempdata = adc.read(MCP3208::Channel::SINGLE_7);

    break;
  /////////////////////////////////////
    case 9:
	tempdata = adc.read(MCP3208::Channel::SINGLE_0 );

    break;
    case 10:
	tempdata = adc.read(MCP3208::Channel::SINGLE_1 );

    break;
    case 11:
	tempdata = adc.read(MCP3208::Channel::SINGLE_2 );

    break;
    case 12:
	tempdata = adc.read(MCP3208::Channel::SINGLE_3 );

    break;
    case 13:
	tempdata = adc.read(MCP3208::Channel::SINGLE_4 );

    break;
    case 14:
	tempdata = adc.read(MCP3208::Channel::SINGLE_5 );

    break;
    case 15:
	tempdata = adc.read(MCP3208::Channel::SINGLE_6 );

    break;
    case 16:
	tempdata = adc.read(MCP3208::Channel::SINGLE_7 );

    break;
  /////////////////////////////////////
    case 17:
	tempdata = adc.read(MCP3208::Channel::SINGLE_0 );

    break;
    case 18:
	tempdata = adc.read(MCP3208::Channel::SINGLE_1 );
 
    break;
    case 19:
	tempdata = adc.read(MCP3208::Channel::SINGLE_2 );

    break;
    case 20:
	tempdata = adc.read(MCP3208::Channel::SINGLE_3 );

    break;
    case 21:
	tempdata = adc.read(MCP3208::Channel::SINGLE_4 );

    break;
    case 22:
	tempdata = adc.read(MCP3208::Channel::SINGLE_5 );

    break;
    case 23:
	tempdata = adc.read(MCP3208::Channel::SINGLE_6 );

    break;
    case 24:
	tempdata = adc.read(MCP3208::Channel::SINGLE_7 );

    break;
  /////////////////////////////////////
    case 25:
	tempdata = adc.read(MCP3208::Channel::SINGLE_0 );

    break;
    case 26:
	tempdata = adc.read(MCP3208::Channel::SINGLE_1 );

    break;
    case 27:
	tempdata = adc.read(MCP3208::Channel::SINGLE_2 );

    break;
    case 28:
	tempdata = adc.read(MCP3208::Channel::SINGLE_3 );

    break;
    case 29:
	tempdata = adc.read(MCP3208::Channel::SINGLE_4 );

    break;
    case 30:
	tempdata = adc.read(MCP3208::Channel::SINGLE_5 );

    break;
    case 31:
	tempdata = adc.read(MCP3208::Channel::SINGLE_6 );

    break;
    case 32:
	tempdata = adc.read(MCP3208::Channel::SINGLE_7 );

    break;
    default:
      beep();
      delay(200);
      beep();
      delay(200);  
      beep();
    break;

  }
  delay(20);
  digitalWrite(IC1_CS, HIGH);
  digitalWrite(IC2_CS, HIGH);
  digitalWrite(IC3_CS, HIGH);
  digitalWrite(IC4_CS, HIGH);

  return tempdata;
   
}

unsigned char readrotary(void)
{
  unsigned char newPos = encoder.getPosition() * ROTARYSTEPS;
  
  if (newPos < ROTARYMIN) {
    //encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    newPos = ROTARYMAX;

  } else if (newPos > ROTARYMAX) {
    //encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    newPos = ROTARYMIN;
  } 
  
  if (pos != newPos) 
  {    
    pos = newPos;   
  }
  return newPos;
}

void readmode(void)
{
  static byte oldmode ;
  if(digitalRead(Mode))
  {
    workmode = STAN;
  }
  else{
    workmode = COM;  
  }
  if(oldmode != workmode) beep();
  oldmode = workmode;
}

void readplaypause(void)
{
  if(digitalRead(Playpausebtn) ==0)
  {
    delay(20); 
    if(digitalRead(Playpausebtn) ==0)
    {
	  beep();
      delay(500); 
      if(play)
      {
        play = 0; 
        
        // Off All Relay
//        digitalWrite(RELAY1_1, HIGH);
//        digitalWrite(RELAY1_2, HIGH);
//        digitalWrite(RELAY2_1, HIGH);
//        digitalWrite(RELAY2_2, HIGH);
//        digitalWrite(RELAY3_1, HIGH);
//        digitalWrite(RELAY3_2, HIGH);
//        digitalWrite(RELAY4_1, HIGH);
//        digitalWrite(RELAY4_2, HIGH);
        
      }
      else
      {
        if((readnum !=0) &&(readnum <=32))
        {
          play = 1;
          loopcount = 1;
          startmeasure = true;
          lcd1.setCursor(0, 3);lcd1.print("Scanning     :");
          cleardisplay();
        }       
      }
      
    }
  }
}

int meaheatsinktemp(void)
{
  int stat = 0;
  int temp = 0; //temp Temp Data
 
  stat = 0;
  // Read Temp Heat sink Left
  if(readnum >= 29){  //29-32
  temp = 0;
  for (byte i = 0; i < 20 ; i++)
  {
    temp +=   analogRead(Temp_R4);
    delay(2);
  }
  temp /= 20;
  if((temp > 0 ) && (temp < 1024))
    //Temp8 = temp;
    
    // Raw Data To Temp Mapping
  if (temp >= T40 )
    Temp8 = 40 ;
  else if((temp >= T45) && (temp < T40))//720-770
    Temp8 = 45 ;          //
  else if((temp >= T50) && (temp < T45))//670-720
    Temp8 = 50 ;          //
  else if((temp >= T55) && (temp < T50))//630-670
    Temp8 = 55 ;          //
  else if((temp >= T60) && (temp < T55))//600-630
    Temp8 = 60 ;          //
  else if((temp >= T65) && (temp < T60))//540-600
    Temp8 = 65 ;          //
  else if((temp >= T70) && (temp < T65))//500-540
    Temp8 = 70 ;          //
  else if((temp >= T75) && (temp < T70))//  480-500
    Temp8 = 75 ;          //  80   75  
  else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp8 = 80 ;          //  85   80
  else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp8 = 85 ;          //  90   85
  else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp8 = 90 ;           // 95   90
  else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp8 = 95 ;
  else if(temp < T95) //380
    Temp8 = 100 ;
    
  }
  if(readnum >= 25){ //25-28
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_R3);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))
    //Temp7 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp7 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp7 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp7 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp7 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp7 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp7 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp7 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp7 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp7 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp7 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp7 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp7 = 95 ;
    else if(temp < T95) //380
    Temp7 = 100 ;

  }
  if(readnum >= 21){ //21-24
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_R2);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))
    //Temp6 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp6 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp6 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp6 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp6 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp6 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp6 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp6 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp6 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp6 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp6 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp6 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp6 = 95 ;
    else if(temp < T95) //380
    Temp6 = 100 ;
    
  }
  if(readnum >= 17){  //17-20
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_R1);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))//Temp >
    //Temp5 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp5 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp5 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp5 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp5 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp5 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp5 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp5 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp5 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp5 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp5 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp5 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp5 = 95 ;
    else if(temp < T95) //380
    Temp5 = 100 ;
  }
  if(readnum >= 13){ // 13-16
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_L4);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))
    //Temp4 = temp;
    // Raw Data To Temp Mapping
   if (temp >= T40)
   Temp4 = 40 ;
   else if((temp >= T45) && (temp < T40))//720-770
   Temp4 = 45 ;         //
   else if((temp >= T50) && (temp < T45))//670-720
   Temp4 = 50 ;         //
   else if((temp >= T55) && (temp < T50))//630-670
   Temp4 = 55 ;         //
   else if((temp >= T60) && (temp < T55))//600-630
   Temp4 = 60 ;         //
   else if((temp >= T65) && (temp < T60))//540-600
   Temp4 = 65 ;         //
   else if((temp >= T70) && (temp < T65))//500-540
   Temp4 = 70 ;         //
   else if((temp >= T75) && (temp < T70))//  480-500
   Temp4 = 75 ;         //  80   75
   else if((temp >= T80) && (temp < T75))// 460 - 480
   Temp4 = 80 ;         //  85   80
   else if((temp >= T85) && (temp < T80))// 440 - 460
   Temp4 = 85 ;         //  90   85
   else if((temp >= T90) && (temp < T85))// 420 - 440
   Temp4 = 90 ;          // 95   90
   else if((temp >= T95) && (temp < T90))// 400 - 420
   Temp4 = 95 ;
   else if(temp < T95) //380
   Temp4 = 100 ;
  }
  if(readnum >= 9){ //9-12
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_L3);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))
    //Temp3 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp3 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp3 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp3 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp3 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp3 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp3 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp3 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp3 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp3 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp3 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp3 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp3 = 95 ;
    else if(temp < T95) //380
    Temp3 = 100 ;
  }
  if(readnum >= 5){ //5-8
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_L2);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))
    //Temp2 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp2 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp2 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp2 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp2 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp2 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp2 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp2 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp2 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp2 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp2 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp2 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp2 = 95 ;
    else if(temp < T95) //380
    Temp2 = 100 ;
  }
  if(readnum >= 1){ //1-4
    temp = 0;
    for (byte i = 0; i < 20 ; i++)
    {
      temp +=   analogRead(Temp_L1);
      delay(2);
    }
    temp /= 20;
    if((temp > 0 ) && (temp < 1024))//Temp >
    //Temp1 = temp;
    // Raw Data To Temp Mapping
    if (temp >= T40)
    Temp1 = 40 ;
    else if((temp >= T45) && (temp < T40))//720-770
    Temp1 = 45 ;          //
    else if((temp >= T50) && (temp < T45))//670-720
    Temp1 = 50 ;          //
    else if((temp >= T55) && (temp < T50))//630-670
    Temp1 = 55 ;          //
    else if((temp >= T60) && (temp < T55))//600-630
    Temp1 = 60 ;          //
    else if((temp >= T65) && (temp < T60))//540-600
    Temp1 = 65 ;          //
    else if((temp >= T70) && (temp < T65))//500-540
    Temp1 = 70 ;          //
    else if((temp >= T75) && (temp < T70))//  480-500
    Temp1 = 75 ;          //  80   75
    else if((temp >= T80) && (temp < T75))// 460 - 480
    Temp1 = 80 ;          //  85   80
    else if((temp >= T85) && (temp < T80))// 440 - 460
    Temp1 = 85 ;          //  90   85
    else if((temp >= T90) && (temp < T85))// 420 - 440
    Temp1 = 90 ;           // 95   90
    else if((temp >= T95) && (temp < T90))// 400 - 420
    Temp1 = 95 ;
    else if(temp < T95) //380
    Temp1 = 100 ;
  }

  // Read Temp Heat sink Right 
  if((Temp1 > MAXTEMP) || (Temp2 > MAXTEMP) || (Temp3 > MAXTEMP) || (Temp4 > MAXTEMP)){
  digitalWrite(FAN1, LOW); //FAN1 ON
  }
  else if((Temp1 < MINTEMP) && (Temp2 < MINTEMP) && (Temp3 < MINTEMP) && (Temp4 < MINTEMP) ){
  digitalWrite(FAN1, HIGH); //FAN1 OFF
  }
  else if((Temp1 > OVERHEAT) || (Temp2 > OVERHEAT) || (Temp3 > OVERHEAT) || (Temp4 > OVERHEAT)){
  digitalWrite(FAN1, LOW); //FAN1 ON
  digitalWrite(BUZZER, LOW); // Test BUZZER ON
  stat = 1;
  }
    
  if((Temp5 > MAXTEMP) || (Temp6 > MAXTEMP) || (Temp7 > MAXTEMP) || (Temp8 > MAXTEMP)){
    digitalWrite(FAN2, LOW); //FAN1 ON
  }
  else if((Temp5 < MINTEMP) && (Temp6 < MINTEMP) && (Temp7 < MINTEMP) && (Temp8 < MINTEMP) ){
    digitalWrite(FAN2, HIGH); //FAN1 OFF
  }
  else if((Temp5 > OVERHEAT) || (Temp6 > OVERHEAT) || (Temp7 > OVERHEAT) || (Temp8 > OVERHEAT)){
    digitalWrite(FAN2, LOW); //FAN1 ON
    digitalWrite(BUZZER, LOW); // Test BUZZER ON
    stat = 1;
  }
  
  return stat;
}

/*************************** Interrupt function timer1 50 ms. ***************************/
ISR (TIMER3_OVF_vect)                           // 1 Tick
{
  static unsigned char k;
  //cbi(TIFR,TOV1);                             //Clear overflow bit timer1
  //TIFR1 &= ~(1 << TOV1);
  //
  //TCNT1H = 0xFF;                // Over flow 50 ms.
  //TCNT1L = 0x77;
  
  TIFR3 &= ~(1 << TOV3);
  
  TCNT3H = 0xFE;                // Over flow 50 ms.
  TCNT3L = 0x77;

  if(++k >= 200){ // 50*200 = 10 sec
    k = 0;
    digitalWrite(13, !digitalRead(13));
    flagreadtemp = 1 ;
  }
  
  return;
}


float calculate_display(uint16_t raw ,uint8_t ch)
{
  uint16_t ohmmilli;
  float    volt,ohm;
  //uint16_t tempdata = 0;
  
//  raw = tempdata ;
  volt = raw * VOLTREAD;
  ohm  = volt/I_CONST;
  
  //Serial.print(ohm);
  //Serial.println(" ohm");
 // ลบ ความต้านทานในสาย
//  if(ohm > 0.2 ){
//    ohm = ohm - 0.19;
//  }
  
  
  float temp = ohm * 1000;
  ohmmilli = (uint16_t)temp;
    
	if((ohmmilli > 100) && (ohmmilli < MAXRESISTANT)) 
	{
		ohmmilli -= 100;
	}
	//Show LCD and LED  
  switch(ch)
  {
    case 1:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(3, 0); lcd2.print("---");
        bitWrite(red1_8, 0, 1); bitWrite(green1_8, 0, 0);
      }
      else{
        lcd2.setCursor(3, 0);
        if(ohmmilli < 10)
        {
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        
        bitWrite(green1_8, 0, 1); bitWrite(red1_8, 0, 0);
      }
    break;
    
    case 5:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(10, 0);lcd2.print("---");
        bitWrite(red1_8, 1, 1); bitWrite(green1_8, 1, 0); 
      }
      else{
        lcd2.setCursor(10, 0);
        if(ohmmilli < 10)
        {  
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 1, 1); bitWrite(red1_8, 1, 0); 
      }
    break;

    case 9:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(17, 0);lcd2.print("---"); 
        bitWrite(red1_8, 2, 1); bitWrite(green1_8, 2, 0);
      }
      else{
        lcd2.setCursor(17, 0);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 2, 1); bitWrite(red1_8, 2, 0);
      }
    break;
    
    case 2:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(3, 1);lcd2.print("---"); 
        bitWrite(red1_8, 3, 1); bitWrite(green1_8, 3, 0);
      }
      else{
        lcd2.setCursor(3, 1);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 3, 1); bitWrite(red1_8, 3, 0);
      }
    break;
    
    case 6:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(10, 1);lcd2.print("---");
       bitWrite(red1_8, 4, 1); bitWrite(green1_8, 4, 0); 
      }
      else{
        lcd2.setCursor(10, 1);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 4, 1); bitWrite(red1_8, 4, 0); 
      }
    break;
    
    case 10:
      if(ohmmilli >= MAXRESISTANT){
        lcd2.setCursor(17, 1);lcd2.print("---");
        bitWrite(red1_8, 5, 1); bitWrite(green1_8, 5, 0); 
      }
      else
      {
        lcd2.setCursor(17, 1);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 5, 1); bitWrite(red1_8, 5, 0);
      }
    break;
    
    case 3:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(3, 2);lcd2.print("---"); 
        bitWrite(red1_8, 6, 1); bitWrite(green1_8, 6, 0);
      }
      else
      {
        lcd2.setCursor(3, 2);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 6, 1); bitWrite(red1_8, 6, 0);
      }
    break;
    
    case 7:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(10, 2);lcd2.print("---");
        bitWrite(red1_8, 7, 1); bitWrite(green1_8, 7, 0);  
      }
      else{
        lcd2.setCursor(10, 2);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green1_8, 7, 1); bitWrite(red1_8, 7, 0); 
      }
    break;
    
    case 11:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(17, 2);lcd2.print("---");
        bitWrite(red9_16, 0, 1); bitWrite(green9_16, 0, 0);  
      }
      else
      {
        lcd2.setCursor(17, 2);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green9_16, 0, 1); bitWrite(red9_16, 0, 0);
      }      
    break;
     
    case 4:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(3, 3);lcd2.print("---"); 
        bitWrite(red9_16, 1, 1); bitWrite(green9_16, 1, 0);
      }
      else
      {
        lcd2.setCursor(3, 3);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        }
        bitWrite(green9_16, 1, 1); bitWrite(red9_16, 1, 0);
      }      
    break;
     
    case 8:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(10, 3);lcd2.print("---"); 
        bitWrite(red9_16, 2, 1); bitWrite(green9_16, 2, 0);
      }
      else
      {
        lcd2.setCursor(10, 3);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        } 
        bitWrite(green9_16, 2, 1); bitWrite(red9_16, 2, 0);
      }      
    break;
    case 12:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd2.setCursor(17, 3);lcd2.print("---"); 
        bitWrite(red9_16, 3, 1); bitWrite(green9_16, 3, 0);
      }
      else
      {
        lcd2.setCursor(17, 3);
        if(ohmmilli < 10)
        {       
          lcd2.print("  ");
          lcd2.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd2.print(" ");
          lcd2.print(ohmmilli);
        }
        else
        {
          lcd2.print(ohmmilli);
        } 
        bitWrite(green9_16, 3, 1); bitWrite(red9_16, 3, 0);
      }      
    break;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    case 13:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(3, 0);lcd3.print("---"); 
        bitWrite(red9_16, 4, 1); bitWrite(green9_16, 4, 0);
      }
      else
      {
        lcd3.setCursor(3, 0);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green9_16, 4, 1); bitWrite(red9_16, 4, 0);
      }      
    break;
    
    case 17:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(10, 0);lcd3.print("---"); 
        bitWrite(red9_16, 5, 1); bitWrite(green9_16, 5, 0);
      }
      else
      {
        lcd3.setCursor(10, 0);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green9_16, 5, 1); bitWrite(red9_16, 5, 0);
      }      
    break;
     
    case 21:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(17, 0);lcd3.print("---"); 
        bitWrite(red9_16, 6, 1); bitWrite(green9_16, 6, 0);
      }
      else
      {
        lcd3.setCursor(17, 0);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green9_16, 6, 1); bitWrite(red9_16, 6, 0);
      }     
    break;
     
    case 14:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(3, 1);lcd3.print("---"); 
        bitWrite(red9_16, 7, 1); bitWrite(green9_16, 7, 0);
      }
      else
      {
        lcd3.setCursor(3, 1);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        } 
        bitWrite(green9_16, 7, 1); bitWrite(red9_16, 7, 0);
      }     
    break;
  ////////////////////////////////////////////////////////////////////////////////////////////
     
    case 18:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(10, 1);lcd3.print("---"); 
        bitWrite(red17_24, 0, 1); bitWrite(green17_24, 0, 0); 
      }
      else
      {
        lcd3.setCursor(10, 1);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 0, 1); bitWrite(red17_24, 0, 0);
      }      
    break;
     
    case 22:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(17, 1);lcd3.print("---"); 
        bitWrite(red17_24, 1, 1); bitWrite(green17_24, 1, 0);
      }
      else
      {
        lcd3.setCursor(17, 1);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 1, 1); bitWrite(red17_24, 1, 0);
      }      
    break;
    
    case 15:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(3, 2);lcd3.print("---"); 
        bitWrite(red17_24, 2, 1); bitWrite(green17_24, 2, 0); 
      }
      else
      {
        lcd3.setCursor(3, 2);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 2, 1); bitWrite(red17_24, 2, 0);
      }      
    break;
    
    case 19:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(10, 2);lcd3.print("---"); 
        bitWrite(red17_24, 3, 1); bitWrite(green17_24, 3, 0);
      }
      else
      {
        lcd3.setCursor(10, 2);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        } 
        bitWrite(green17_24, 3, 1); bitWrite(red17_24, 3, 0);
      }      
    break;
    
    case 23:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(17, 2);lcd3.print("---"); 
        bitWrite(red17_24, 4, 1); bitWrite(green17_24, 4, 0);
      }
      else
      {
        lcd3.setCursor(17, 2);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 4, 1); bitWrite(red17_24, 4, 0);
      }      
    break;
     
    case 16:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(3, 3);lcd3.print("---"); 
        bitWrite(red17_24, 5, 1); bitWrite(green17_24, 5, 0);
      }
      else
      {
        lcd3.setCursor(3, 3);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 5, 1); bitWrite(red17_24, 5, 0);
      }      
    break;
     
    case 20:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(10, 3);lcd3.print("---"); 
        bitWrite(red17_24, 6, 1); bitWrite(green17_24, 6, 0);
      }
      else
      {
        lcd3.setCursor(10, 3);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        }
        bitWrite(green17_24, 6, 1); bitWrite(red17_24, 6, 0);
      }      
    break;
    case 24: 
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd3.setCursor(17, 3);lcd3.print("---"); 
        bitWrite(red17_24, 7, 1); bitWrite(green17_24, 7, 0);
      }
      else
      {
        lcd3.setCursor(17, 3);
        if(ohmmilli < 10)
        {       
          lcd3.print("  ");
          lcd3.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd3.print(" ");
          lcd3.print(ohmmilli);
        }
        else
        {
          lcd3.print(ohmmilli);
        } 
        bitWrite(green17_24, 7, 1); bitWrite(red17_24, 7, 0);
      }      
    break;
  ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 25:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(3, 0);lcd4.print("---"); 
        bitWrite(red25_32, 0, 1); bitWrite(green25_32, 0, 0);
      }
      else
      {
        lcd4.setCursor(3, 0);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 0, 1);; bitWrite(red25_32, 0, 0);
      }      
    break;
    
    case 28:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(10, 0);lcd4.print("---"); 
        bitWrite(red25_32, 1, 1); bitWrite(green25_32, 1, 0);
      }
      else
      {
        lcd4.setCursor(10, 0);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 1, 1); bitWrite(red25_32, 1, 0);
      }      
    break;
    
    case 31:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(17, 0);lcd4.print("---"); 
        bitWrite(red25_32, 2, 1); bitWrite(green25_32, 2, 0);
      }
      else
      {
        lcd4.setCursor(17, 0);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 2, 1); bitWrite(red25_32, 2, 0);
      }      
    break;
    
    case 26:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(3, 1);lcd4.print("---"); 
        bitWrite(red25_32, 3, 1); bitWrite(green25_32, 3, 0);
      }
      else
      {
        lcd4.setCursor(3, 1);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 3, 1); bitWrite(red25_32, 3, 0);
      }    
    break;
    case 29: 
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(10, 1);lcd4.print("---"); 
        bitWrite(red25_32, 4, 1); bitWrite(green25_32, 4, 0);
      }
      else
      {
        lcd4.setCursor(10, 1);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 4, 1); bitWrite(red25_32, 4, 0);
      }      
    break;
    //case 30: 
    case 32:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(17, 1);lcd4.print("---"); 
        bitWrite(red25_32, 5, 1); bitWrite(green25_32, 5, 0); 
      }
      else
      {
        lcd4.setCursor(17, 1);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        } 
        bitWrite(green25_32, 5, 1); bitWrite(red25_32, 5, 0);
      }      
    break;
    
    case 27:
      if(ohmmilli >= MAXRESISTANT)
      {
        lcd4.setCursor(3, 2);lcd4.print("---"); 
        bitWrite(red25_32, 6, 1); bitWrite(green25_32, 6, 0); 
      }
      else
      {
        lcd4.setCursor(3, 2);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        } 
        bitWrite(green25_32, 6, 1); bitWrite(red25_32, 6, 0);
      }    
    break;
    
    case 30:
      if(ohmmilli >= MAXRESISTANT){
        lcd4.setCursor(10, 2);lcd4.print("---"); 
        bitWrite(red25_32, 7, 1); bitWrite(green25_32, 7, 0);
      }
      else{
        lcd4.setCursor(10, 2);
        if(ohmmilli < 10)
        {       
          lcd4.print("  ");
          lcd4.print(ohmmilli);
        }
        else if((ohmmilli < 100) && (ohmmilli > 9))
        {
          lcd4.print(" ");
          lcd4.print(ohmmilli);
        }
        else
        {
          lcd4.print(ohmmilli);
        }
        bitWrite(green25_32, 7, 1); bitWrite(red25_32, 7, 0);
      }
    break;
    
    default:
      beep();
      delay(200);
      beep();
      delay(200);  
      beep();
    break;
      
  }
  if(ch == readnum)
    {
      switch(ch)
      {
        case 1:
          bitWrite(red1_8, 3, 0); bitWrite(green1_8, 3, 0);//2
          bitWrite(red1_8, 6, 0); bitWrite(green1_8, 6, 0);//3
          bitWrite(red9_16, 1, 0);bitWrite(green9_16, 1, 0);//4
          bitWrite(red1_8, 1, 0); bitWrite(green1_8, 1, 0);//5
          bitWrite(red1_8, 4, 0); bitWrite(green1_8, 4, 0);//6
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(3, 1);   lcd2.print("---"); //2
          lcd2.setCursor(3, 2);   lcd2.print("---"); //3
          lcd2.setCursor(3, 3);   lcd2.print("---"); //4
          lcd2.setCursor(10, 0);  lcd2.print("---"); //5
          lcd2.setCursor(10, 1);  lcd2.print("---"); //6
          lcd2.setCursor(10, 2);  lcd2.print("---"); //7
          lcd2.setCursor(10, 3);  lcd2.print("---"); //8
          lcd2.setCursor(17, 0);  lcd2.print("---"); //9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 2:
          bitWrite(red1_8, 6, 0); bitWrite(green1_8, 6, 0);//3
          bitWrite(red9_16, 1, 0);bitWrite(green9_16, 1, 0);//4
          bitWrite(red1_8, 1, 0); bitWrite(green1_8, 1, 0);//5
          bitWrite(red1_8, 4, 0); bitWrite(green1_8, 4, 0);//6
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14 
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(3, 2);   lcd2.print("---"); //3
          lcd2.setCursor(3, 3);   lcd2.print("---");//4
          lcd2.setCursor(10, 0);  lcd2.print("---");//5
          lcd2.setCursor(10, 1);  lcd2.print("---");//6
          lcd2.setCursor(10, 2);  lcd2.print("---");//7
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32   
        break;
        case 3:
          bitWrite(red9_16, 1, 0);bitWrite(green9_16, 1, 0);//4
          bitWrite(red1_8, 1, 0); bitWrite(green1_8, 1, 0);//5
          bitWrite(red1_8, 4, 0); bitWrite(green1_8, 4, 0);//6
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(3, 3);   lcd2.print("---");//4
          lcd2.setCursor(10, 0);  lcd2.print("---");//5
          lcd2.setCursor(10, 1);  lcd2.print("---");//6
          lcd2.setCursor(10, 2);  lcd2.print("---");//7
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 4:
          bitWrite(red1_8, 1, 0); bitWrite(green1_8, 1, 0);//5
          bitWrite(red1_8, 4, 0); bitWrite(green1_8, 4, 0);//6
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(10, 0);  lcd2.print("---");//5
          lcd2.setCursor(10, 1);  lcd2.print("---");//6
          lcd2.setCursor(10, 2);  lcd2.print("---");//7
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 5:
          bitWrite(red1_8, 4, 0); bitWrite(green1_8, 4, 0);//6
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(10, 1);  lcd2.print("---");//6
          lcd2.setCursor(10, 2);  lcd2.print("---");//7
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 6:
          bitWrite(red1_8, 7, 0); bitWrite(green1_8, 7, 0);//7
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(10, 2);  lcd2.print("---");//7
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        break;
        case 7:
          bitWrite(red9_16, 2, 0);bitWrite(green9_16, 2, 0);//8

          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(10, 3);  lcd2.print("---");//8
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 8:
          bitWrite(red1_8, 2, 0);bitWrite(green1_8, 2, 0);//9
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd2.setCursor(17, 0);  lcd2.print("---");//9
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
      /////////////////////////////////////
        case 9:
          bitWrite(red1_8, 5, 0);bitWrite(green1_8, 5, 0);//10
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd2.setCursor(17, 1);  lcd2.print("---");//10
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 10:
          bitWrite(red9_16, 0, 0);bitWrite(green9_16, 0, 0);//11
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd2.setCursor(17, 2);  lcd2.print("---");//11
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 11:
          bitWrite(red9_16, 3, 0);bitWrite(green9_16, 3, 0);//12
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd2.setCursor(17, 3);  lcd2.print("---");//12

          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 12:
          bitWrite(red9_16, 4, 0);bitWrite(green9_16, 4, 0);//13
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(3, 0);   lcd3.print("---");//13
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 13:
          bitWrite(red9_16, 7, 0);bitWrite(green9_16, 7, 0);//14
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(3, 1);   lcd3.print("---");//14
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 14:
          bitWrite(red17_24, 2, 0);bitWrite(green17_24, 2, 0);//15
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(3, 2);   lcd3.print("---");//15
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 15:
          bitWrite(red17_24, 5, 0);bitWrite(green17_24, 5, 0);//16

          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(3, 3);   lcd3.print("---");//16
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 16:
          bitWrite(red9_16, 5, 0);bitWrite(green9_16, 5, 0);//17
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(10, 0);  lcd3.print("---");//17
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
      /////////////////////////////////////
        case 17:
          bitWrite(red17_24, 0, 0);bitWrite(green17_24, 0, 0);//18
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(10, 1);  lcd3.print("---");//18
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 18:
          bitWrite(red17_24, 3, 0);bitWrite(green17_24, 3, 0);//19
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(10, 2);  lcd3.print("---");//19
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 19:
          bitWrite(red17_24, 6, 0);bitWrite(green17_24, 6, 0);//20
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(10, 3);  lcd3.print("---");//20
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 20:
          bitWrite(red9_16, 6, 0);bitWrite(green9_16, 6, 0);//21
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(17, 0);  lcd3.print("---");//21
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 21:
          bitWrite(red17_24, 1, 0);bitWrite(green17_24, 1, 0);//22
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(17, 1);  lcd3.print("---");//22
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 22:
          bitWrite(red17_24, 4, 0);bitWrite(green17_24, 4, 0);//23
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(17, 2);  lcd3.print("---");//23
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 23:
          bitWrite(red17_24, 7, 0);bitWrite(green17_24, 7, 0);//24

          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd3.setCursor(17, 3);  lcd3.print("---");//24

          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 24:
          bitWrite(red25_32, 0, 0);bitWrite(green25_32, 0, 0);//25
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(3, 0);   lcd4.print("---");//25
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
      /////////////////////////////////////
        case 25:
        
          bitWrite(red25_32, 3, 0);bitWrite(green25_32, 3, 0);//26
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(3, 1);   lcd4.print("---");//26
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
          
        break;
        case 26:
          bitWrite(red25_32, 6, 0);bitWrite(green25_32, 6, 0);//27
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(3, 2);   lcd4.print("---");//27
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 27:
          bitWrite(red25_32, 1, 0);bitWrite(green25_32, 1, 0);//28
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(10, 0);  lcd4.print("---");//28
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 28:
          bitWrite(red25_32, 4, 0);bitWrite(green25_32, 4, 0);//29
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(10, 1);  lcd4.print("---");//29
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 29:
          bitWrite(red25_32, 7, 0);bitWrite(green25_32, 7, 0);//30
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(10, 2);  lcd4.print("---");//30
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 30:
          bitWrite(red25_32, 2, 0);bitWrite(green25_32, 2, 0);//31
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          
          lcd4.setCursor(17, 0);  lcd4.print("---");//31
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 31:
          bitWrite(red25_32, 5, 0);bitWrite(green25_32, 5, 0);//32
          lcd4.setCursor(17, 1);  lcd4.print("---");//32
        break;
        case 32:
        
        break;
        default:
          beep();
          delay(200);
          beep();
          delay(200);  
          beep();
        break;

      }
      
    }
	return ohm;
  
}
// end of file
