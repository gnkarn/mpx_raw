/*
Author: AnalysIR
Revision: 1.0

This code is provided to overcome an issue with Arduino IR libraries
It allows you to capture raw timings for signals longer than 255 marks & spaces.
Typical use case is for long Air conditioner signals.

You can use the output to plug back into IRremote, to resend the signal.

This Software was written by AnalysIR.

Usage: Free to use, subject to conditions posted on blog below.
Please credit AnalysIR and provide a link to our website/blog, where possible.

Copyright AnalysIR 2014

Please refer to the blog posting for conditions associated with use.
http://www.analysir.com/blog/2014/03/19/air-conditioners-problems-recording-long-infrared-remote-control-signals-arduino/

Connections:
IR Receiver      Arduino
V+          ->  +5v
GND          ->  GND
Signal Out   ->  Digital Pin 2
(If using a 3V Arduino, you may connect V+ to +3V)
2018 _ Raw_ir modified by GNK to decode mpx 16 bit code
*/

#define LEDPIN 13
//you may increase this value on Arduinos with greater than 2k SRAM
#define maxLen 34

// Uncomment to enable printing out nice debug messages.
#define DEBUG

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial


// Setup debug printing macros.
#ifdef DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

volatile  uint32_t irBuffer[maxLen]; //stores timings - volatile because changed by ISR
uint16_t delta[maxLen]; // contiene los tiempos netos de cada pulso en lugar del tiempo absoluto
volatile unsigned int x = 0; //Pointer thru irBuffer - volatile because changed by ISR


uint8_t data[2]; //vector de resultado
#define BitOneTicks 2400  // si es mayor que 2400 ms es un uno
#define BitZeroTicks 1400  // si es menor que 1400 ms es un cero



void setup() {
  Serial.begin(115200); //change BAUD rate as required
  attachInterrupt(0, rxIR_Interrupt_Handler, CHANGE);//set up ISR for receiving IR signal
}

void loop() {
  // put your main code here, to run repeatedly:

  //Serial.println(F("Press the button on the remote now - once only"));
  delay(1000); // pause 5 secs

  if (x==34) { //if a signal is captured
    digitalWrite(LEDPIN, HIGH);//visual indicator that signal received
    Serial.println();
    Serial.print(F("Raw: (")); //dump raw header format - for library
    Serial.print((x - 1));
    Serial.print(F(") "));
    detachInterrupt(0);//stop interrupts & capture until finshed here

    for (unsigned int i = 1; i < x; i++) { //now dump the times
      if (!(i & 0x1)) {
        Serial.print(F("-"));

      }

      Serial.print(irBuffer[i] - irBuffer[i - 1]);
      delta[i-1]= irBuffer[i] - irBuffer[i - 1];

      Serial.print(F(", "));
    }

    x = 0;
    Serial.println();
    Serial.println();
    decode_mpx(); // imprime resultado
    digitalWrite(LEDPIN, LOW);//end of visual indicator, for this time
    attachInterrupt(0, rxIR_Interrupt_Handler, CHANGE);//re-enable ISR for receiving IR signal


  }

}

void rxIR_Interrupt_Handler() {
  if (x > maxLen) return; //ignore if irBuffer is already full
  irBuffer[x++] = micros(); //just continually record the time-stamp of signal transitions

}

// Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
int  decode_mpx(){
    // Reset 16 bits of received data to zero.
  data[0] = data[1]  = 0;
    for (int i=0; i<16; ++i) {
      uint32_t lowCycles  = delta[2*i];
     uint32_t highCycles = delta[2*i+1];


      if ((highCycles == 0)) {
        DEBUG_PRINTLN(F("Timeout waiting for pulse."));
        //_lastresult = false;
        return; //_lastresult;
      }
      data[i/8] <<= 1;
      // Now compare the low and high cycle times to see if the bit is a 0 or 1.
      if (highCycles > BitOneTicks) {
        // High cycles are greater than 50us low cycle count, must be a 1.
        data[i/8] |= 1;
        //DEBUG_PRINTLN(highCycles); // debug  sacar

      }
      // Else high cycles are less than (or equal to, a weird case) the 50us low
      // cycle count so this must be a zero.  Nothing needs to be changed in the
      // stored data.

    }
    DEBUG_PRINT(data[0]); DEBUG_PRINT("_");DEBUG_PRINTLN(data[1]); // debug gnk
    DEBUG_PRINT(data[0], HEX);DEBUG_PRINT("_");DEBUG_PRINTLN(data[1], HEX); // debug gnk
  }
