/*-----------------------------------------------------------------------------

Arduino Digital Input

Monitor digital inputs, then emit a serial character depending on the pin
status. The character will be uppercase for pin connected (N.O. button push)
and lowercase for pin disconnected (N.O. button released). It will begin with
"A" for the first I/O pin, "B" for the next, and so on. Currently, with pins
0 and 1 used for serial TX/RF, this leaves pins 2-12 available (10), with pin
13 reserved for blinking the onboard LED as a heartbeat "we are alive"
indicator.

This software was written to allow an Arduino act as a cheap input/trigger
interface to software such as VenueMagic. As I do not own a copy of this
software, I could only test it under the 15 day trial. There may be other
issues...

2012-10-09 0.0 allenh - Initial version.
2012-10-11 0.1 allenh - Updated debounce to work with timing rollover, via:
                        http://www.arduino.cc/playground/Code/TimingRollover
                        Fixed bug where last DI pin was not being used.

-----------------------------------------------------------------------------*/
//#include <EEPROM.h>
#include <avr/wdt.h>

/* TX/RX pins are needed to USB/serial. */
#define DI_PIN_START  2
#define DI_PIN_END    12
#define DI_PIN_COUNT  (DI_PIN_END-DI_PIN_START+1)

#define LED_PIN 13
#define LEDBLINK_MS 1000
#define DEBOUNCE_MS 100 // 100ms (1/10th second)

/*---------------------------------------------------------------------------*/
/*
 * Some sanity checks to make sure the #defines are reasonable.
 */
#if (DI_PIN_END >= LED_PIN)
#error PIN CONFLICT: PIN END goes past LED pin.
#endif

#if (DI_PIN_START < 2)
#error PIN CONFLICT: PIN START covers 0-TX and 1-RX pins.
#endif

#if (DI_PIN_START > DI_PIN_END)
#error PIN CONFLICT: PIN START and END should be a range.
#endif
/*---------------------------------------------------------------------------*/

/* For I/O pin status and debounce. */
unsigned int  pinStatus[DI_PIN_COUNT];      // Last set PIN mode.
unsigned long pinDebounce[DI_PIN_COUNT];    // Debounce time.
unsigned int  debounceRate = DEBOUNCE_MS;   // Debounce rate.
unsigned long pinCounter[DI_PIN_COUNT];

/* For the blinking LED (heartbeat). */
unsigned int  ledStatus = LOW;             // Last set LED mode.
unsigned long ledBlinkTime = 0;            // LED blink time.
unsigned int  ledBlinkRate = LEDBLINK_MS;  // LED blink rate.

unsigned int pinsOn = 0;

/*---------------------------------------------------------------------------*/

void setup()
{
  // Just in case it was left on...
  wdt_disable();
  // Initialize watchdog timer for 2 seconds.
  wdt_enable(WDTO_4S);
  
  // Initialize the pins and pinStatus array.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Set pin to be digital input using pullup resistor.
    pinMode(thisPin+DI_PIN_START, INPUT_PULLUP);
    // Read and save the current pin status.
    pinStatus[thisPin] = HIGH; //digitalRead(thisPin+DI_PIN_START);
    // Clear debounce time.
    pinDebounce[thisPin] = 0;

    pinCounter[thisPin] = 0;
  }

  // Set pin 13 to output, since it has an LED we can use.
  pinMode(LED_PIN, OUTPUT);

  // Initialize the serial port.
  Serial.begin(9600);

  // Docs say this isn't necessary for Uno.
  while(!Serial) {
    ;
  }

  // Emit some startup stuff to the serial port.
  Serial.println("ArduinoDI by Allen C. Huffman (alsplace@pobox.com)");
  Serial.print("Configured for: ");
  Serial.print(debounceRate);
  Serial.print("ms Debounce, ");
  Serial.print(DI_PIN_COUNT);
  Serial.print(" DI Pins (");
  Serial.print(DI_PIN_START);
  Serial.print("-");
  Serial.print(DI_PIN_END);
  Serial.println(").");
  Serial.println("(Nathaniel is a jerk.)");
}

/*---------------------------------------------------------------------------*/

void loop()
{
  // Tell the watchdog timer we are still alive.
  wdt_reset();
  
  // LED blinking heartbeat. Yes, we are alive.
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    if (ledStatus==LOW)  // If LED is LOW...
    {
      ledStatus = HIGH;  // ...make it HIGH.
    } else {
      ledStatus = LOW;   // ...else, make it LOW.
    }
    // Set LED pin status.
    if (pinsOn==0) digitalWrite(LED_PIN, ledStatus);
    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+ledBlinkRate;
  }
  
  // Check for serial data.
  if (Serial.available() > 0) {
    // If data ready, read a byte.
    int incomingByte = Serial.read();
    // Parse the byte we read.
    switch(incomingByte)
    {
      case '?':
        showStatus();
        break;
      default:
        break;
    }
  }
  
  // Loop through each Digital Input pin.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    int status = digitalRead(thisPin+DI_PIN_START);

    // In pin status has changed from our last toggle...
    if (status != pinStatus[thisPin])
    {
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (pinDebounce[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        pinDebounce[thisPin] = millis()+debounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (pinDebounce[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        if ( (long)(millis()-pinDebounce[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status==LOW)
            {
              // Emit UPPERCASE "On" character.
              Serial.println(char(65+thisPin));
              pinCounter[thisPin]++;
              pinsOn++;
              digitalWrite(LED_PIN, HIGH);
            } else {
              // Emit lowercase "Off" character.
              Serial.println(char(97+thisPin));
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            pinStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            pinDebounce[thisPin] = 0;
        } // End of if ( (long)(millis()-pinDebounce[thisPin]) >= 0 )
        
      } // End of if (pinDebounce[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      pinDebounce[thisPin] = 0;
    }
  } // End of for()
}

void showStatus()
{
  int status = 0;
  
  Serial.print("DI: ");

  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    int status = digitalRead(thisPin+DI_PIN_START);
    Serial.print(thisPin+DI_PIN_START);
    Serial.print("=");
    Serial.print(digitalRead(thisPin+DI_PIN_START));
    Serial.print(" ");
  }
  Serial.println("");

  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    Serial.print(thisPin+DI_PIN_START);
    Serial.print(":");
    Serial.print(pinCounter[thisPin]);
    Serial.print(" ");
  }
  Serial.println("");
 
  //Serial.print("millis() = ");
  //Serial.println(millis());
}
/*---------------------------------------------------------------------------*/
// End of file.

