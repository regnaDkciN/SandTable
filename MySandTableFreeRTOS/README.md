Upgrading RP2350 Based Sand Table to FreeRTOS
===
![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)


This document describes the firmware design and code changes that were made to the [RP2350 based Sand Table](https://github.com/regnaDkciN/SandTable/tree/main/MySandTable2350) to use the [FreeRTOS](https://www.freertos.org/) real time operating system (RTOS).

## Motivation
In my opinion, replacing the sand table's Arduino UNO with an [Adafruit Metro RP2350](https://www.adafruit.com/product/6003) is well worth the extra expense.  The UNO provides acceptable basic operation, but its severely limited memory size and processor speed leaves little room for expansion and upgrades.  Using the RP2350 eliminates these problems, but the *MySandTable2350* software version does not take advantage of the RP2350's two core design.  For example, the potentiometers and serial port are polled and serviced while waiting for moves to complete.  This leads to some motion glitching (i.e. not being as smooth as it could be).  

FreeRTOS offers many features that can improve the sand table performance and take better advantage of the RP2350 processor architecture.  For instance:
* Code may be easily assigned to prioritized tasks which can run concurrently and take advantage of what would otherwise be processor idle time.
* Code may easily be assigned to run on either of the RP2350 cores, thus sharing the execution burden between the two cores.
* Software timers (alarms) may be used to generate the interrupts that handle servo updates.
* Tasks may communicate with each other via queues.  This allows tasks to eliminate most busy/wait loops and free up CPU cycles.
* Tasks can easily synchronize with each other as needed via several types of semaphores and other mechanisms.

## System Redesign
Some major changes to the software design were needed In order to take advantage of the RP2350's two cores and FreeRTOS's multitasking capabilities.  To understand the changes, it is necessary to understand the original design.

### Original Design (Before)
The original software was implemented as a single execution loop complemented by two interrupt service routines (ISRs) that directly controlled the rotary and in/out servos.

### New Design (After)

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


