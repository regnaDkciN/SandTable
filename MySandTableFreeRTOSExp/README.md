Upgrading RP2350 Based Sand Table to FreeRTOS
===
![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)


This document describes the firmware design and code changes that were made to the [RP2350 based Sand Table](https://github.com/regnaDkciN/SandTable/tree/main/MySandTable2350) to use the [FreeRTOS](https://www.freertos.org/) real time operating system (RTOS).

## Motivation
In my opinion, replacing the sand table's Arduino UNO with an [Adafruit Metro RP2350](https://www.adafruit.com/product/6003) is well worth the extra expense.  The UNO provides acceptable basic operation, but its severely limited memory size and processor speed leaves little room for expansion and upgrades.  Using the RP2350 eliminates these problems, but the existing system design does not take advantage of the RP2350's two core design.  For example, the potentiometers and serial port are polled while waiting for moves to complete.  This leads to some motion glitching (i.e. not being as smooth as it could be).


## Use Default FreeRTOSConfig.h

## Do Not Call vTaskStartScheduler()

## Debug / Safety Net Code

## loop() Priority

## Move Serial and ADC Handling to Core 1

## Redo Polling Loops

## Add Volatile

## Critical Section Handling

## Random Seed Setup

## Protect delayMicroseconds()

## Use PIOs
The latest version of the sand table firmware uses the the PIO state machines of the RP2350 processor to generate servo step pulses.  New file - STStepper.pio and STStepper.pio.h - implement the servo stepping logic.  Each step now uses the state machine to do the following for each step:
1. Set the servo's direction.
2. Delay 1 uSec.
3. Toggle the step pin high.
4. Delay 6 uSec.
5. Toggle the step pin low.
6. Delay 6 uSec.

As a result of using the PIO state machines, the servo ISR's were modified to step on each call rather than on every other call.  This allows the ISR to be called half as often as before, which leaves more execution time for other things.

The Arduino IDE does not directly handle compiling the STStepper.pio file to the STStepper.pio.h file.  In order to generate the STStepper.pio.h file from the STStepper.pio file, the STStepper.pio file was added to a Visual Studio Code project, it was compiled, then the generated STStepper.pio.h file was copied to the MySandTableFreeRTOS directory.






        
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



  ### Execution Speed Differences
The rotary servo ISR contains logic to generate compensation pulses in the in/out servo proportional to the rotary axis movement.  Part of this logic uses a short delay to affect a pulse on the in/out servo.  In the UNO, a pulse delay of 1 microsecond was used.  Due to the slow speed of the UNO processor, this delay plus the surrounding code was sufficient to generate a pulse long enough for the servo driver hardware to handle.  However, since the RP2350 is so much faster, it was feared that the generated pulse might be too short, so the pulse delay was changed to 5 microseconds.  This was probably not necessary, but was done just in case



  ### GPIO Differences
The Arduino UNO firmware uses D12 as the enable for the rotation axis.  However, the Metro RP2350 uses D12 for HSTX connectivity.  The pin that used to be occupied by D12 is now D22.  As a result, the RP2350 code was changed to initialize D22 as the rotation axis enable output instead of D12.



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


