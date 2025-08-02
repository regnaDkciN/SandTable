Upgrading from Arduino UNO to Adafruit Metro RP2350 (A Case Study)
===
![Headline](https://i.imgur.com/OjllJqS.jpeg)

This document describes the code and hardware changes that were needed to migrate a non-trivial Arduino UNO application to the Adafruit Metro RP2350 board.  Most readers will be familiar with the Arduino UNO, so it will not be described here.  The Adafruit Metro RP2350 is very close to a drop-in replacement for the Arduino UNO, but with a few differences that need to be accounted for.

The Adafruit Metro RP2350 is a tremendous improvement in processor speed and memory size over the Arduino UNO.  Here is a comparison of some of the more important features of each board:

|    |  Arduino UNO |  Adafruit Metro RP2350  |
| :---------- | :---------- | :-------- |
| CPU | ATMega328P | Dual Cortex M33 or Dual RISC-V
| CPU Voltage | 5 V | 3.3 V |
| CPU Speed | 16 MHz | 150 MHz |
| Bus Width | 8 Bits | 32 Bits |
| Flash Memory | 32 KB | 16 MB |
| RAM | 2 KB | 528 KB |
| Floating Point Processor (FPU) | No | Yes|
| Programmable I/O (PIO) | 0 | 12 |
| GPIO | 20 | 23 | 
| Analog Inputs | 6 | 6 |
| Micro SD Card Slot | No | Yes |
| Onboard RGB Neo Pixel | No | Yes |
| USB Type | USB B | USB C |


## The Application
A recent Instructable presented the [World's First Cycloid Art Table: How I Built This Arduino-Powered Spirograph Machine by NewsonsElectronics](https://www.instructables.com/Worlds-First-Cycloid-Art-Table-How-I-Built-This-Ar/). 

![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)

 This sand table uses laser cut parts and is driven by an Arduino UNO.  I had been looking for an affordable sand table project and this one fit the bill.  During the construction of the sand table, I noted a few areas that needed clarification, and some areas for improvement.  These are documented [here](https://github.com/regnaDkciN/SandTable).  One of the major issues I noted was that the Arduino UNO is somewhat under powered for this application with it's relatively slow processor and very limited memory size.  For example, my updated Arduino program for the sand table used nearly 100% of both flash and RAM, leaving no room for the future addition of shapes or other firmware improvements.  In addition, the calculations to generate certain shapes caused some some jerky servo motion which was partially due to the slow Arduino UNO.

## Adafruit Metro RP2350 to the Rescue
The Adafruit Metro RP2350 has the same physical size and shape as the Arduino UNO, and has (almost)  the same pinout.  More on that later.  This allows the Metro to work with the Arduino CNC shield without changing almost anything.  The following sections describe the changes, some required and some not, that I made to the sand table firmware and hardware.  I am quite pleased with the results.

### Data Type Changes
Since the UNO was tight on space, all data types were originally declared as the smallest size that would do the job in the UNO program.  In most cases ```(u)int16_t``` was used since the UNO compiler uses 16-bit integers as its native int size.  Using the native integer size saves on both memory size and execution time since using a different sized integer requires the compiler to generate and execute extra code.

The RP2530 that is used by the Metro has lots of memory compared to the UNO and its native integer size is 32-bits.  Therefore, when using the RP2350,  memory size is unimportant, and execution speed becomes the major goal.  As a result, almost all integer data types were changed to fast types.  For example, ```uint16_t``` becomes ```uint_fast16_t```, etc.  The _fast_ data types allow the compiler to use an integer size that is as least as large as the specified size, but may be larger if larger integers are faster on the particular machine.  In the case of the RP2350, since the native integer size is 32-bits, a ```(u)int_fast16_t``` will actually use 32-bits, but the same compiled for the Arduino UNO will use 16-bits.

### Speed Delay Value Changes
The UNO program set the timer clock period to 4 microseconds (uSec).  The default RP2350 timer clock period is 1 uSec.  To account for this, all speed delay values were multiplied by 4.  For example ```SPEED_DELAY_MIN_VAL``` was changed from 25 on the UNO to 100 on the RP2350, etc.  Speed increments in  ```HandleRemoteCommands()```  were also adjusted by a factor of 4.


### Changed Servos from 8 Microsteps to 64 Microsteps
With the RP2350's speed and native 32 bit integers, it was possible to improve the servo microstepping from 8 to 64.  This provides smoother and quieter servo movement.  However, this also now requires that the fastest servo moves occur at a 25 microsecond rate.  This is far too fast for the Arduino UNO, but the RP2350 can handle it with no problems.

Both hardware and firmware changes were required for this improvement:
* Hardware wise, a jumper was added to the M1 pins under each of the TMC2209 driver boards.  This selects 64 step microstepping.  This may be a bit confusing since the Arduino CNC shield and the TMC2209 use different labeling for the microstepping pins.   The pin that the Arduino CNC shield labels M1 actually corresponds to the pin that the TMC2209 refers to as MS2.  In any case, the jumper needs to be placed on the middle microstep pin of the Arduino CNC shield .

	![Microstep Jumper](https://i.imgur.com/FapHoFK.jpeg)
* Firmware wise,three changes were needed:
	-  All step related variables and constants needed to change to 32 bit integers since the maximum step count now exceeds the limits of a 16 bit integer.
	-  All step related constants needed to be multiplied by 8 to account for the 8 times increase in microstepping.	
	-  All speed related delay constants needed to be further divided by 8 to account for the microstepping change.

```
const int_fast32_t  ROT_TOTAL_STEPS   = 8 * 16000;  // Rotation axis total steps.
const int_fast32_t  INOUT_TOTAL_STEPS = 8 * 4300;   // In/Out axis total steps.
const uint_fast32_t HOME_ROT_OFFSET   = 347 * (ROT_TOTAL_STEPS / 1000);
...
volatile int_fast32_t InOutSteps   = 0;      // Current # steps in/out is away from 0.
int_fast32_t          InOutStepsTo = 0;      // Number inout steps needed to reach target.
volatile int_fast32_t RotSteps     = 0;      // Current # steps rotary is away from 0.
int_fast32_t          RotStepsTo   = 0;      // Number rotary steps needed to reach target.
...
const int_fast16_t  SPEED_DELAY_MIN_VAL = 100 / 8;   // Minimum axis moving delay value (uSec).
const int_fast16_t  SPEED_DELAY_MAX_VAL = 2000 / 8;  // Maximum axis speed delay value (uSec).

```


### Different Timer Handling
The UNO needs the timer clock rate to be set up before use as follows:
```
        TCCR1A = 0;           // Init Timer1A (for rotation motor).
        TCCR1B = 0;           // Init Timer1B (for in/out motor).

        TCCR1B |= B00000011;  // Prescaler = 64. (4 microsecond tick).
        OCR1A = 1000;         // Timer Compare1A Register.
        OCR1B = 1000;         // Timer Compare1B Register.

        TIMSK1 |= B00000110;  // Only use one interupt.
```

The RP2350 already has its timer running at 1MHz, so doesn't require any further initialization.


### Different Interrupt Handling
The UNO used a simple interrupt setup with ```ISR(vector)``` indicating that an interrupt will be handled by the function.  For example, the following outline shows how the UNO Interrupt Service Routines ( ISRs) were defined:
```
        // Timer 1A interrupt handler.
        ISR(TIMER1_COMPA_vect)
        {
            OCR1A = TCNT1 + RotDelay;
            ...
            ISR code
            ...
        }
        
        // Timer 1B interrupt handler.
        ISR(TIMER1_COMPB_vect)
        {
            OCR1B = TCNT1 + InOutDelay;
            ...
            ISR code
            ...
        }
```


The RP2350 handles timer interrupts more normally in that the ISR must be declared before use, and added to the timer list before it can be used as follows:
```
        int64_t RotaryServoIsr(__unused alarm_id_t id, __unused void *user_data);
        int64_t InOutServoIsr(__unused alarm_id_t id, __unused void *user_data);
        
        .
        .
        .
        
        void setup()
        {
        	 . . .
        	add_alarm_in_us(1000000, RotaryServoIsr, NULL, false);
       	 	add_alarm_in_us(1000000, InOutServoIsr,  NULL, false);
        	. . .
        }
        
        // Rotary servo ISR.
        int64_t RotaryServoIsr(__unused alarm_id_t id, __unused void *user_data)
        {
            ...
            ISR code
            ...
            return -RotDelay;
        }
        
        // Linear servo ISR.
        int64_t InOutServoIsr(__unused alarm_id_t id, __unused void *user_data)
        {
            ...
            ISR code
            ...
            return -InOutDelay;
        }
```

In order for the interrupt to repeat periodically, the UNO bumps the corresponding ```OCR1x``` register value at the start of each ISR as shown above.  For the RP2350 ISR to repeat, it returns the delay value from its ISR as a negative value, as shown above, indicating that the next execution should start exactly that many microseconds after the time the current ISR was triggered.

### Random Seed Generation
The latest UNO implementation of the sand table used an open analog input and a function - ```GenerateRandomSeed()``` - to generate a random seed at startup.  The RP2350 has a hardware based true random number generator with a standard function - ```get_rand_32()``` - to generate a 32-bit  random number.  So ```GenerateRandomSeed()``` was removed for the RP2350 version.

  ### API Math Function Differences
The Arduino UNO API supplies a ```squaref()``` function to square a float value.  This function is replaced in the RP2350 API with a function named ```sq()``` that does the same thing. So uses of ```squaref()``` were changed to ```sq()```.

  ### Execution Speed Differences
The rotary servo ISR contains logic to generate compensation pulses in the in/out servo proportional to the rotary axis movement.  Part of this logic uses a short delay to affect a pulse on the in/out servo.  In the UNO, a pulse delay of 1 microsecond was used.  Due to the slow speed of the UNO processor, this delay plus the surrounding code was sufficient to generate a pulse long enough for the servo driver hardware to handle.  However, since the RP2350 is so much faster, it was feared that the generated pulse might be too short, so the pulse delay was changed to 5 microseconds.  This was probably not necessary, but was done just in case

**UPDATE:** The latest version of the sand table firmware uses the the PIO state machines of the RP2350 processor to generate servo step pulses.  New file - STStepper.pio and STStepper.pio.h - implement the servo stepping logic.  Each step now uses the state machine to do the following for each step:
1. Set the servo's direction.
2. Delay 1 uSec.
3. Toggle the step pin high.
4. Delay 6 uSec.
5. Toggle the step pin low.
6. Delay 6 uSec.

As a result of using the PIO state machines, the servo ISR's were modified to step on each call rather than on every other call.  This allows the ISR to be called half as often as before, which leaves more execution time for other things.

The Arduino IDE does not directly handle compiling the STStepper.pio file to the STStepper.pio.h file.  In order to generate the STStepper.pio.h file from the STStepper.pio file, the STStepper.pio file was added to a Visual Studio Code project, it was compiled, then the generated STStepper.pio.h file was copied to the MySandTable2350 directory.

  ### GPIO Differences
The Arduino UNO firmware uses D12 as the enable for the rotation axis.  However, the Metro RP2350 uses D12 for HSTX connectivity.  The pin that used to be occupied by D12 is now D22.  As a result, the RP2350 code was changed to initialize D22 as the rotation axis enable output instead of D12.

### Added Pause Indication
The Metro RP2350 board contains a red LED that comes in handy in the sand table application to indicate that the axes are pausing, i.e. when the speed knob is turned to its lowest level, or paused via a remote serial command.  This was a simple change and provides some welcome feedback to the user.
```
const int PAUSE_LED_PIN      = PIN_LED;     // Pause LED lights red when paused.
...
bool UpdateSpeeds()
{
     ...
      // Indicate if we're pausing.
    digitalWrite(PAUSE_LED_PIN, !keepRunning);
    ...
}
...
void setup()
{
    ...
    pinMode(PAUSE_LED_PIN, OUTPUT);
    ...
}
```

  ### Serial Logging
The SerialLog.h file provided the ability to selectively log data to the serial port based on the value of user selectable macros.  The major problem with the UNO implementation is that the API did not provide a ```printf()``` function, and the ```sprintf()``` function provided by the UNO API did not handle 64-bit integers or floating point numbers.  As a result, many places in the sand table code resorted to clumsy sequences of log generation.  For example, the following code appears in the UNO version of the ```PlotShapeArray()``` function:
```
            // Display the current index and position.
            LOG_F(LOG_STEP, "%d   ", i);
            LOG_U(LOG_STEP, x);
            LOG_U(LOG_STEP, ",");
            LOG_U(LOG_STEP, y);
            LOG_U(LOG_STEP, "\n");
```

This was necessitated due to the UNO ```sprintf()` function not supporting floats.  The RP2350 API provides a fully operational ```printf()``` function that can handle all data types.  As a result, the SerialLog.h file was replaced with SerialLog2350.h which handles the logs more gracefully.  For example, the above 5 log statements were replace with the following in the RP2350 code:
```
            // Display the current index and position.
            LOG_F(LOG_STEP, "%d   %.1f,%.1f\n", i, x, y);
```


### Temporarily Handle Compiler Floating Point Issue
**NOTE:  As of 02-AUG-2025 this issue has been fixed on all RP2350 versions of the sand table software.  This requires version 4.6.1 or later of the Raspberry Pi Pico/RP2040/RP2350 API by Earle F. Philhower, III.**

I initially spent about an hour trying to find out why certain shape paths were not being generated correctly in my first try of the RP2350 code.  I eventually discovered that some floating point trig operations were sometime returning incorrect values.  After a web search, I came across the [RP2350 FPU compiler issue #2429](https://github.com/raspberrypi/pico-sdk/pull/2429).  It looks like this issue has been fixed on a development branch, but has not been released to production code yet.  Until it is fixed, my workaround is to change all floats to doubles, and change all float function calls to their double equivalent.  This naturally makes the code execution slower, so I will undo these changes once a good version of the compiler is released.  Here are the changes that will be needed:

| Double Function | Float Function |
| --- | --- |
| ```double``` |  ```float_t``` |
| ```cos()``` |  ```cosf()``` |
| ```sin()``` | ```sinf()``` |
| ```hypot()``` | ```hypotf()``` |
| ```atan2()``` | ```atan2f()``` |
| ```round()``` | ```roundf()``` |
| ```trunc()```| ```truncf()``` |
| ```sqrt()``` | ```sqrtf()``` |
| ```pow()```  | ```powf()``` |
 
### 3.3 Volt vs 5 Volt Processor Differences
The Arduino UNO is a 5 volt device.  The RP350 runs at 3.3 volts.  The original design fed the speed and brightness pots from the UNO's 5V source.  While the Metro RP2350 has a 5V output pin, it should not be used with the analog inputs (or GPIO pins for that matter).  The simple change was to move the 5V pot connection to the CNC shield 3.3V pin (next to the RST pin).

### Make MOSFET Handle 3.3V
The MOSFET used for the UNO version (IRFZ44N) has a Gate Threshold Voltage (VGS) of 2 - 4 volts.  This is a little too high for the 3.3V used by the RP2350.  The MOSFET would not fully turn on with the 3.3V gate.  The IRFZ44N was replaced with the [IRLZ44N](https://www.amazon.com/dp/B0CBKH4XGL) which has a VGS of 1-2 volts.  This allowed the MOSFET to turn on fully.  Also added a 100K resistor between the gate and source to keep the MOSFET from conducting at power-up before ```setup()``` could run.

### Work Around Analog Input Noise
Significant noise was found on the RP2350 analog inputs.  This showed up as LED flicker at lower brightness levels.  To compensate for this, the pot readings (analog inputs) were configured with a resolution of 12 bits, then shifted right by 4 bits after reading.  This effectively produced an 8 bit resolution, which is fine for the sand table application, but eliminated most of the noise/flicker issue.
 
Also changed the PWM (LED) output frequency to 100,000 to help reduce flicker.

```
// Potentiometer related constants.
const int_fast32_t  PWM_FREQ            = 100000; // PWM frequency.
const int_fast16_t  ANALOG_RESOLUTION   = 12;     // Number of ADC bits.
const int_fast16_t  ANALOG_SHIFT_FACTOR = 4;      // Amount to shift analog pot readings
                                                  // to mitigate ADC jitter.

setup()
{
    ...
    // We set the PWM frequency to 100 khz and set the ADC resolution to 12 bits
    // (4096 counts) to help reduce LED flicker at low LED knob settings.
    analogWriteFreq(PWM_FREQ);
    analogReadResolution(ANALOG_RESOLUTION);
    ...
}

inline uint_fast16_t ReadAPot(int pot)
{
    return analogRead(pot) >> ANALOG_SHIFT_FACTOR;
} // End ReadAPot().
```
### Account for Physical Differences
The Metro RP2350 includes an SD card holder and a power switch among other hardware that the UNO doesn't have.  These are physically located at the edges of the card.  The 3D printed bumper used to mount the card to the sand table was modified in order to allow the use of the SD card and power switch if desired.  The "Metro_2350_Bumper" files reflect these changes.

![Notches](https://i.imgur.com/ES8oE8q.jpeg)

The Arduino UNO uses a USB-B connector while the Metro RP2350 uses a USB-C connector.  This required the face plate USB hole to be slightly lowered and re-sized to accommodate a USB-C connector.  The face plate USB hole was lowered by 2 mm and its height reduced to 10 mm.  The "Faceplate 2350" files reflect the changes.
