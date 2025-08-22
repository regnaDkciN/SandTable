Upgrading RP2350 Based Sand Table to FreeRTOS
===
![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)

**As of 21-AUG-2025 this branch will no longer be updated, but is kept for historical design info.**

This document describes the firmware design and code changes that were made to the [RP2350 based Sand Table](https://github.com/regnaDkciN/SandTable/tree/main/MySandTable2350) to use the [FreeRTOS](https://www.freertos.org/) real time operating system (RTOS).

## Motivation
In my opinion, replacing the sand table's Arduino UNO with an [Adafruit Metro RP2350](https://www.adafruit.com/product/6003) is well worth the extra expense.  The UNO provides acceptable basic operation, but its severely limited memory size and processor speed leaves little room for expansion and upgrades.  Using the RP2350 eliminates these problems, but the *MySandTable2350* software version does not take advantage of the RP2350's two core design.  For example, the potentiometers and serial port are polled and serviced while waiting for moves to complete.  This leads to some motion glitching (i.e. not being as smooth as it could be).  

FreeRTOS offers many features that can improve the sand table performance and take better advantage of the RP2350 processor architecture.  For instance:
* Code may be easily assigned to prioritized tasks that can run concurrently and take advantage of what would otherwise be processor idle time.
* Code may easily be assigned to run on either of the RP2350 cores, thus sharing the execution burden between the two cores.
* Software timers (alarms) may be used to generate the interrupts that handle servo updates.
* Tasks may communicate with each other via queues (and other mechanisms).  This allows tasks to eliminate most busy/wait loops and free up CPU cycles.
* Tasks can easily synchronize with each other as needed via several types of semaphores and other mechanisms.

## System Redesign
Some major changes to the software design were needed In order to take advantage of the RP2350's two cores and FreeRTOS's multitasking capabilities.  To understand the changes, it is necessary to understand the original design.

### Original Design (Before)
The original software was implemented as a single execution loop complemented by two interrupt service routines (ISRs) that directly controlled the rotary and in/out servos.   ```MotorRatio()``` moves are strictly handled by the ISRs, so there is no problem with them.  However, all other moves suffer from possible motion glitching as shown in the timing diagram below:


<img src="https://i.imgur.com/VfguGsQ.png" width=1024>

The above timing diagram attempts to show a greatly simplified and exaggerated  timing diagram of the original sand table software.  Important points are:
* The bottom timeline is used for timing reference and has no absolute relation to actual system timing.
* The top line (ISRs) represents ISR execution.  The sand table software actually uses two ISRs, one for rotary, and one for in/out.  The diagram shows a single ISR.  This is sufficient to explain the motion glitch behavior. In the diagram, the ISRs run periodically every 20 timing units.  When there is no motion to execute, they return quickly.  When motion is in process execution takes longer.
* The loop() line shows a greatly simplified ```loop()``` function behavior.  For our purposes, we are only concerned with move generation and polling for motion completion.  It only executes when the ISRs are not running.
* The timing starts with motion ISRs idle and ```loop()``` generating the next move.
* At time 30, the first move calculation has completed and the ISRs are commanded to start the move.   At this time, ```loop()``` starts polling the pots to update the speed and brightness values, checks the serial port for incoming requests, and starts polling for motion completion.
* At time 40 the ISRs see the move request and begin executing the move. ```loop()``` continues updating speed and brightness values, checking serial requests, and checking for completion of the move.
* This repeats with the ISRs performing the move and the ```loop()``` code polling until at time 86 the ISRs signal that the move has completed.  At this point, ```loop()``` is in the middle of its polling, which does not complete until time 110 at which time it begins calculations for the next move.  By this time, the ISRs have executed and seen no motion request, so simply return without doing anything.
* The ```loop()``` function takes some time generating the position for the next move, and doesn't make the next move request until time 150.  By this time several more idle ISR times have elapsed, which further disturb the smoothness of the motion.
* The cycle repeats after this.

It can be seen from the above that there is room for improvement.

### New Design (After)
#### Motion
The redesign takes advantage of both of the RP2350 cores as well as FreeRTOS multitasking.  The code has been broken up into the following tasks:
* Shape Task - This task selects random shapes and generates and executes shape moves.  It executes on core 0 at a priority level of 5.  This priority was selected to be the highest on core 0 so that the ```loop()``` function that executes at priority 4 won't interfere with (preempt) it.
* Print Task - This task handles all outgoing serial port traffic.  All serial logging and status data is sent via this task.  It executes on core 1 at a priority of 2.  This task shares its priority with the Remote Command Handler Task.  Both execute infrequently, and neither has critical timing requirements.
* Remote Command Handler Task - This task waits for incoming serial data and handles any command requests.  It executes on core 1 at a priority of 2.
* Read Pots Task - This task continuously polls the potentiometers (pots) and updates the servo speed and LED brightness.  It executes on core 1 at a priority of 0.  The priority of this task is low.  It executes when nothing else on core 1 is running.  It shares its priority with the idle task so that the idle task can get a chance to run periodically.

<img src=https://i.imgur.com/DYPdfTB.jpeg width=400>

With this setup, all of the things that needed to be polled in the original design are moved to the second core while keeping all motion related items on the first core.  This eliminates most of the need for the shape generation code to poll while waiting for motion to complete.

As with most real-time operating systems, FreeRTOS allows a task to wait on an event without the need to poll for it.  In FreeRTOS, the Task Notification API provides an efficient method for a task to wait for an event to occur.  This mechanism is used in the new sand table design.  After the Shape Task completes the generation of the next target position, it calls ```xTaskNotifyWait()``` which causes the task to enter the pending state until the move completes.  For example, the code for the ```MoveTo()``` function was changed to:
```
void MoveTo(float_t x, float_t y)
{
   // Make sure we start with the servo complete flag inactive.
    xTaskNotifyStateClear(ShapeHandle);

    // Calculate the target in/out and rotary values and start moving.
    ReverseKinematics(x, y);

    // Wait until the target is reached.  Periodically check for abort.
    while ((xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1000)) != pdTRUE) && !AbortShape)
    {
        // Do nothing.
    }
} // End MoveTo().
```
Note that in this implementation, we still wake up periodically to check for an abort.  The wait loop continues until ```xTaskNotifyWait()``` returns ```pdTrue```, which indicates that the move has completed, or until an abort is commanded.

When the servos reach the target position, they signal the Shape Task that the move is complete via a call to ```xTaskNotifyFromISR()``` as follows:
```
            if (RotSteps == RotStepsTo)
            {
                // Turn off the motor if it reaches the set point.
                RotOn = false;
            }
            if (InOutSteps == InOutStepsTo)
            {
                InOutOn = false;
            }
            if (!RotOn && !InOutOn)
            {
                xTaskNotifyFromISR(ShapeHandle, 0, eNoAction, &taskWoken);
            }
```
The timing of the new new motion related design is shown below: 


<img src=https://i.imgur.com/yZ1QpDW.png width=1024>

Again, the above diagram is greatly simplified and exaggerated.  The important points to note are:
* The timing starts with motion ISRs idle and the Shape Task generating the next move.
* At time 30, the Shape Task has completed generating the next target position and starts to pend on motion completion (i.e. it goes to sleep).
* At time 40, the ISRs see that new motion is needed, and starts moving the axes.
* At time 88, the move completes, and the ISRs notify the Shape Task.  The shape task wakes up as soon as the ISR completes.
* The Shape Task begins generating the next move.
* At time 110, the Shape Task completes its next target position calculations and starts the next move.  It then starts to pend on motion completion again.
* The cycle repeats after this.

Notice that there is still a small motion glitch with this design, but it is smaller than the original design's glitch.  The next version of the software - MySandTableFreeRTOSExp.ino - handles this.

#### Serial Logging
A new task - Print Task - was created to handle sending of all serial output.  This task resides on core 1 and uses a queue - Print Queue -  as its input.  Any task in the system, on either core, may request a message to be sent out the serial port by sending a message to the Print Queue via the ```LOG_F()``` macro.

<img src=https://i.imgur.com/7pNqpiO.jpeg width=600>

The SerialLog2350.h file was replaced by SerialLogFreeRTOS.h and SerialLogFreeRTOS.cpp.  These files implement a new SerialLogFreeRTOS class that acts as an interface between Print Task and the rest of the system via the Print Queue.  The helper macro ```LOG_F()``` is still used, but behind the scenes it calls the ```SerialLogFreeRTOS::PrintLog()``` method to affect the serial output.  

The SerialLogFreeRTOS class contains the following methods:
* Constructor - ```SerialLogFreeRTOS(size_t bufSize, QueueHandle_t q)``` with ```bufSize``` specifying the size in bytes of the largest expected string to be output, and ```q``` specifying a queue that was already created via the FreeRTOS ```xQueueCreate()``` function.
* Destructor - ```~SerialLogFreeRTOS()``` which cleans up when a SerialLogFreeRTOS object is deleted.  The sand table software never deletes a SerialLogFreeRTOS object.
* ```PrintLog(const char *pFmt, ...)``` - Takes a printf style format string and arguments, converts them to a C string, then sends the string to the print task for outputting.

In use, the Print Task simply creates a queue, then creates a SerialLogFreeRTOS object, and waits in an infinite loop for the print queue to have a string to send out the serial port.  The entire task is as follows:
```
void PrintTask(__unused void *param)
{
    // Create our print queue and log object.
    QueueHandle_t q = xQueueCreate(PRINT_QUEUE_SIZE, sizeof(char *));
    SerialLogFreeRTOS log(MAX_LOG_STRING_SIZE * PRINT_QUEUE_SIZE, q);
    char *pData;

    // Wait to receive an outgoing message request, then send it out the serial port.
    while (1)
    {
        const uint_fast32_t PRINT_TIMEOUT_MS = 10000;
        if (xQueueReceive(q, &pData, pdMS_TO_TICKS(PRINT_TIMEOUT_MS)) == pdPASS)
        {
            Serial.print(pData);
        }
    }
} // End PrintTask().

```
The system should never directly call ```PrintLog()``` though.  Instead, the ```LOG_F()``` macro should be used.  It is defined in SerialLogFreeRTOS.h as follows:
```
#define LOG_F(level, format, ...)                                   \
    do                                                              \
    {                                                               \
        if(level)                                                   \
        {                                                           \
            SerialLogFreeRTOS::PrintLog(format , ##__VA_ARGS__);    \
        }                                                           \
    } while (0)
```

## Use Default FreeRTOSConfig.h
The FreeRTOS documentation contains a lot of information pertaining to tailoring FreeRTOSConfig.h macros to the specific system's needs.  In most cases the settings are meant to conserve memory on limited systems.  The default FreeRTOSConfig.h file for the RP2350 basically enables all features.  Since the RP2350 contains ample memory, there is no need to modify it.  Just Include FreeRTOS.h in the .ino  file and all the FreeRTOS features become available.

## Do Not Call vTaskStartScheduler()
The FreeRTOS documentation shows that a call to ```vTaskStartScheduler()``` is required after tasks are created in  order to start the scheduler.  However, for the RP2350 version, this is unnecessary since, I assume, that the scheduler is started automatically upon exit from ```setup()```.  It was found that when an explicit call to  ```vTaskStartScheduler()```  was added at the end of ```setup()```, the system crashed. **BEWARE:  DO NOT CALL ```vTaskStartScheduler()``` WHEN USING THE RP2350.**

## Debug / Safety Net Code
When using FreeRTOS, it is easy to make a coding mistake that will cause the system to crash in such a way that you can no longer download to the RP2350 board.  I found this out the hard way and had to go through the factory reset procedure.  The procedure is not difficult, but  due to the need to simultaneously activate 2 push buttons on the board, and the physical placement of the board in the sand table, this was something I didn't want to repeat.  To handle this, the code below was added in ```setup()```.  It waits until the speed pot is not zero.  I would recommend a similar setup to anyone experimenting with FreeRTOS.
```
    // Special code for use while debugging.  Hold startup until the speed pot
    // is no longer in the pausing state.  If something we did caused FreeRTOS
    // to go into the weeds, this will allow our USB connection to be seen by
    // the host computer and allow us to recover.
    while (ReadAPot(SPEED_POT_PIN) <= KNOB_MIN_VAL + 16) { /* Do nothing.*/ }
```


## loop() Priority 4
The new sand table design did not need to use the ```loop()``` function since its main loop is executed in the Shape Task.   However, it was found that the ```loop()``` function must be defined.  The sand table ```loop()``` function simply contains a delay to allow the idle task to execute. It was also found that the ```loop()``` executes at priority 4.

## Add Volatile
Since the system was partitioned into separate concurrently executing tasks, several of the global variables needed to be assigned as ```volatile```.  Among these changes were:
```
volatile bool         Pausing      = false;  // 'true' when pausing.
volatile bool         AbortShape   = false;  // 'true' if aborting ghe current shape.
volatile bool         RandomSeedChanged = false;  // 'true' when random seed has been changed.
```

## Critical Section Handling
A few sections of code needed protection to insure that several lines of code were performed atomically without being preempted by ISRs or other tasks.  Previously, the ```noInterrupts()``` and ```interrupts()``` functions were used for this.  However, for FreeRTOS, the ```taskENTER_CRITICAL()``` and ```taskEXIT_CRITICAL()``` functions are used to do the same thing.  Most cases where critical sections are needed are to start both servos at the same time.  For example:
```
    // Start the move by turning the motors on atomically.
    taskENTER_CRITICAL();
    RotOn   = true;
    InOutOn = true;
    taskEXIT_CRITICAL();
``` 
Other places include handling some of the remote commands.  In addition, several functions in the sand table software use the ```delayMicroseconds()``` function to generate short execution delays.  It was found that these delays were not very accurate under FreeRTOS.  Protecting these calls via critical sections seemed to fix the problem.  For example:
```
        taskENTER_CRITICAL();
        InOutStepper.Step(DirInOut);
        delayMicroseconds(delay);
        taskEXIT_CRITICAL();
```

## Random Seed Setup
The ```randomSeed()``` function is used to initialize the random number generator to a specific value.  At startup, the sand table code generates a random seed and feeds it to ```randomSeed()```.  It was found that when ```randomSeed()``` was called from ```setup()``` its new value was not reflected in the Shape Task.  It is known that some of the Arduino libraries do not work well with FreeRTOS, and the random number generator is one of them.  My conjecture is that each task has its own random number seed.  My workaround was to move the setting of the random number seed to the start of the Shape Task since this is the only task that uses random numbers.

