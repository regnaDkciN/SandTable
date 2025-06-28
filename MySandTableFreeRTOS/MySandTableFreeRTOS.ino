/////////////////////////////////////////////////////////////////////////////////
// MySandTable2350.ino
//
// This file consists of an Arduino sketch that is used to control the sand table
// titled "World's First Cycloid Art Table: How I Built This Arduino-Powered
// Spirograph Machine" by NewsonsElectronics.  It was published on Instructables at:
// https://www.instructables.com/Worlds-First-Cycloid-Art-Table-How-I-Built-This-Ar/
//
// The sketch supports the Arduino Uno in conjuction with a CNC shield as described
// on Instructables.  The sketch is based on the original working10iteration.ino
// file but very heavily modified.  This sketch differs from the original as
// follows:
//   - Re-formatted, including renaming variables and functions, using consistent
//     indenting, and generally cleaning up the code.
//   - Uses Y and Z axes of the CNC shield instead of X and Y of the original.
//     This was due to the X-axis hardware on my CNC shield being damaged.
//   - Used some slightly different algorithms in a few spots.
//   - Added several more shapes, and added randomization functions for each shape.
//   - Many more changes to fix anomalous behavior and enhance operation.
//
// History:
// - 28-JUN-2025 JMC
//   - Major changes to work with FreeRTOS.  Moved all serial and analog handling
//     to core 1.  Kept main path and shape generation coded on core 0.
//   - Fixed reading from the serial port in HandleRemoteCommandsTask().
// - 19-MAY-2025 JMC
//   - Adjusted some delay values.
//   - Fixed RotateToAngle() to rotate in closest direction.
//   - Fixed pause LED to light when remotely paused too.
// - 18-MAY-2025 JMC
//   - Added use of PIO state machines (STStepper) to step the servos.
// - 12-MAY-2025 JMC
//   - Added pause indication via red LED.
//   - Made servo microsteps selectable via MICROSTEPS constant.
// - 27-APR-2025 JMC
//   - Original from MySandTable.ino (Arduino UNO version).  Ported to work
//     with Adafruit Metro 2350 board.  Changes include:
//       + Using fast data types since 2350 is not tight on memory.
//       + Use 1 uSec alarm timer for ISRs (UNO used 4uSec timer).
//       + Converted ISRs for 2350.
//       + Use 2340 get_rand32() to generate starting random seed.
//       + Changed squaref() to sq() due to API differences.
//       + Changed rotary axis enable GPIO to D22 since the Metro uses D12 for
//         other functions.
//       + Changed serial logging to use single printf like macro.
//       + Reworked analog pot reading to throw away 4 LSB's due 2350's noise
//         A2D issue.
//       + Update LEDs while homing.
//       + Temporarily changed all floats and float operations to doubles to
//         work around a 2350 FPU compiler issue #2429:
//         https://github.com/raspberrypi/pico-sdk/pull/2429
//         The issue has been fixed on the development branch, but not added
//         to the release branch yet.  When it is released, this sand table
//         code will restore operation with floats, as double precision is not
//         needed.
//
// Copyright (c) 2025, Joseph M. Corbett
/////////////////////////////////////////////////////////////////////////////////
#include <limits.h>             // For UINT_MAX.
#include <FreeRTOS.h>           // For FreeRTOS core.
#include <task.h>               // For FreeRTOS tasks.
#include <list.h>               // For FreeRTOS lists.
#include <queue.h>              // For FreeRTOS queues.
#include <timers.h>             // For FreeRTOS software timers.
#include <event_groups.h>       // For FreeRTOS event groups.
#include <stream_buffer.h>      // For FreeRTOS stream_buffers.
#include <semphr.h>             // For FreeRTOS semaphores.
#include "SerialLogFreeRTOS.h"  // For data logging macro (LOG_F).
#include "STStepper.pio.h"      // For STStepper class and stepper PIO state machine.


/////////////////////////////////////////////////////////////////////////////////
// S E R I A L   L O G   S E T T I N G S
//
// The following #defines are used in conjunction with SerialLog.h to selectively
// enable/disable printing of certain information.  Setting any macro to 'true'
// enables the specific level of data to be printed.  Setting any macro to 'false'
// disables printing of the specific data level.
/////////////////////////////////////////////////////////////////////////////////
#define LOG_DEBUG        false      // Log debug related info. Not normally  needed.
#define LOG_VERBOSE      false      // Log verbose info. Not normally needed.
#define LOG_INFO         true       // Log some useful extra info.
#define LOG_STEP         false      // Log data to help stepping ShapeArrays
#define LOG_CYCLES       true       // Log useful info regarding remaining cycles.
#define LOG_ALWAYS       true       // Log data unconditionally.


/////////////////////////////////////////////////////////////////////////////////
// User settable constants.
/////////////////////////////////////////////////////////////////////////////////
#define PAUSE_ON_DONE  false // Set to 'true' to pause when drawing finishes.
#define ROT_CAUSING_IN CCW   // Set to 'CW' if clockwise rotation causes IN movement.
                             // Set to 'CCW' if counter clockwise rotation causes IN movement.
#define USE_HOME_SENSOR true // Set to 'true' if using rotary home sensor.

// GPIO pin assignments.
const int EN_INOUT_PIN       = 8;           // Y azis (in/out) enable pin.
const int STEP_INOUT_PIN     = 3;           // Y axis (in/out) step pin.
const int DIR_INOUT_PIN      = 6;           // Y axis (in/out) direction pin.
const int EN_ROT_PIN         = 22;          // Z axis (rotation) enable pin.
const int STEP_ROT_PIN       = 4;           // Z axis (rotation) step pin.
const int DIR_ROT_PIN        = 7;           // Z axis (rotation) direction pin.
const int LIGHTS_PIN         = 11;          // Lights (LEDs) outpuut pin.
const int PAUSE_LED_PIN      = PIN_LED;     // Pause LED lights red when paused.
const int BRIGHTNESS_POT_PIN = A0;          // Brightness pot pin.
const int SPEED_POT_PIN      = A1;          // Speed pot pin.
const int ROT_HOME_PIN       = 5;           // Home reed switch input pin.

// Hardware related constants.
const int_fast32_t  MICROSTEPS        = 64;     // Microsteps used by servos.
const int_fast32_t  ROT_TOTAL_STEPS   = 2000 * MICROSTEPS;
                                                // Rotation axis total steps.
const int_fast32_t  INOUT_TOTAL_STEPS = 537 * MICROSTEPS;
                                                // In/Out axis total steps.
const int_fast16_t  GEAR_RATIO        = 10;     // Ratio of rotary (big) gear to inout
                                                // (small) gear.
const double       MAX_SCALE_F       = 100.0;  // Maximum X or Y coordinate value.
const uint_fast32_t MAX_SCALE_I       = (uint_fast32_t)MAX_SCALE_F;
                                                // Maximum X or Y coordinate value.
const double       STEPS_PER_UNIT    = 1.0;    // Higher values produce smoother moves
                                                // Good values range from 1.0 to 10.0.
const uint_fast32_t HOME_ROT_OFFSET   = 353 * (ROT_TOTAL_STEPS / 1000);
                                                // The rotational offset to be applied
                                                // after detecting the rotary home switch
                                                // while homing.
const int_fast16_t  SPEED_DELAY_MIN_VAL = 1600 / MICROSTEPS;
                                                 // Minimum axis moving delay value (uSec).
                                                 // This macro limits the maximum speed.
                                                 // Original code used 200.  100 is obviously
                                                 // twice as fast, but it is also twice
                                                 // as noisy.  The speed knob can always
                                                 // be turned down to reduce noise if
                                                 // desired, or this value could be
                                                 // increased to 200 or 240.
const int_fast16_t  SPEED_DELAY_MAX_VAL = 32000 / MICROSTEPS;
                                                 // Maximum axis speed delay value (uSec).
const uint_fast16_t MIN_FORCE_DELAY   = SPEED_DELAY_MIN_VAL;
                                                // Minimum servo update delay when forcing.
const uint_fast16_t MAX_FORCE_DELAY   = SPEED_DELAY_MAX_VAL;
                                                // Maximum servo update delay when forcing.
const double       WIPE_RATIO        = 9.4;    // This constant should be changed based on
                                                // ball size.  9.4 is a good value for use
                                                // with 3mm cylindrical magnet.
const uint_fast16_t WIPE_RASTER_INC   = 4;      // This constant should be changed based on
                                                // ball size.  4 is a good value for use
                                                // with 3mm cylindrical magnet.
const uint_fast32_t REMOTE_TIMEOUT_MS = 10000;  // Remote command timeout (milliseconds).

// Useful constants.
const uint_fast8_t IN              = 1;          // In/Out axis IN direction.
const uint_fast8_t OUT             = 0;          // In/Out axis OUT direction.
const uint_fast8_t CW              = 0;          // Rotation axis CLOCKWISE direction.
const uint_fast8_t CCW             = 1;          // Rotation axis COUNTERCLOCKWISE direction.
const double      PI_X_2          = PI * 2.0;   // Useful in many trig calculations.
const double      FLOAT_PRECISION = 100.0;      // Use 2 significant digits for random floats.

// Potentiometer related constants.
const int_fast32_t  PWM_FREQ            = 100000; // PWM frequency.
const int_fast16_t  ANALOG_RESOLUTION   = 12;    // Number of ADC bits.
const int_fast16_t  ANALOG_SHIFT_FACTOR = 4;     // Amount to shift analog pot readings
                                                 // to mitigate ADC jitter.
const uint_fast16_t KNOB_MIN_VAL        = 0;     // Minimum analog reading for knob.
const uint_fast16_t KNOB_MAX_VAL        = (1 << (ANALOG_RESOLUTION - ANALOG_SHIFT_FACTOR)) - 1;
                                                // Maximum analog reading for knob.
const int_fast16_t  BRIGHTNESS_MIN_VAL  = 0;     // Minimum LED brightness value .
const int_fast16_t  BRIGHTNESS_MAX_VAL  = 255;   // Maximum LED brightness value.

// Shape algorithm limits.
const uint_fast16_t MAX_CYCLES      = 50;        // Maximum cycles to generate when drawing.
const uint_fast16_t MAX_STAR_POINTS = 40;        // Max number of Star() points.
const uint_fast16_t MIN_STAR_POINTS = 3;         // Min number of Star() points.
const double       MAX_STAR_RATIO  = 0.95;      // Max ratio of Star() inside to outside points.
const double       MIN_STAR_RATIO  = 0.1;       // Min ratio of Star() inside to outside points.

const uint_fast16_t MAX_POLY_SIDES  = 8;         // Max number of polygon sides.
const uint_fast16_t MIN_POLY_SIDES  = 3;         // Min number of polygon sides.
const uint_fast16_t MIN_POLY_SIZE   = MAX_SCALE_I / 4;
                                                 // Min size of a polygon or star.

const double    MAX_MOTOR_RATIO = WIPE_RATIO;   // Max motor ratio value.
const double    MIN_MOTOR_RATIO = 1.0 / MAX_MOTOR_RATIO;
                                                 // Min motor ratio value.

const uint_fast16_t MIN_CIRCLE_SIZE = MAX_SCALE_I / 4; // Min circle size.
const uint_fast16_t MAX_CIRCLE_SIZE = MAX_SCALE_I;
                                                 // Max non-lobed circle size.
const uint_fast16_t MIN_CIRCLE_LOBES= 1;         // Min circle lobes.
const uint_fast16_t MAX_CIRCLE_LOBES= 10;        // Max circle lobes.

const uint_fast16_t MIN_SPIRO_FIXEDR= MAX_SCALE_I / 3;
                                                 // Minimum spirograph fixed R size.
const uint_fast16_t MIN_SPIRO_SMALLR= 8;         // Minimum spirograph moving circle radius.
const uint_fast16_t SPIRO_NUM_POINTS= 300;       // Number of points to step for
                                                 // spirograph shapes.
const double    SPIRO_ANGLE_BASE= PI_X_2 / (double)SPIRO_NUM_POINTS;
                                                 // Base angle for spirograph shapes.

const uint_fast16_t MIN_ROSE_VAL    = 1;         // Minimum rose shape num and denom values.
const uint_fast16_t MAX_ROSE_VAL    = 20;        // Maximum rose shape num and denom values.
const uint_fast16_t MIN_ROSE_SIZE   = MAX_SCALE_I / 4;
                                                 // Minimum rose x/y size.
const uint_fast16_t MIN_ROSE_RES    = 2;         // Minimum smoothness resolution.
const uint_fast16_t MAX_ROSE_RES    = 64;        // Maximum smoothness resolution.

const uint_fast16_t MIN_CLOVER_VAL  = 1;         // Minimum clover shape radius values.
const uint_fast16_t MAX_CLOVER_VAL  = 10;        // Maximum clover shape radius values.
const uint_fast16_t MIN_CLOVER_SIZE = MAX_SCALE_I / 4;
                                                 // Minimum clover x/y size.
const uint_fast16_t MIN_CLOVER_RES  = 6;         // Minimum clover resolution values.
const uint_fast16_t MAX_CLOVER_RES  = 180;       // Minimum clover resolution values.

const uint_fast16_t MIN_ELLIPSE_SIZE = MAX_SCALE_I / 4;
                                                 // Minimum ellipse x-axis size;
const double    MIN_ELLIPSE_RATIO = 1.3;        // Minimum ellipse ratio.
const double    MAX_ELLIPSE_RATIO = 8.0;        // Mzximum ellipse ratio.

const uint_fast16_t MIN_SERIES_STEPS = 1;        // Minimum number of series steps.
const uint_fast16_t MAX_SERIES_STEPS = 12;       // Maximum number of series steps.
const uint_fast16_t MIN_SERIES_INC   = 8;        // Maximum size increment for series.
const double    MAX_SERIES_ANGLE = 20.0;        // Maximum angle increment for series.

const uint_fast16_t MIN_HEART_SIZE   = MAX_SCALE_I / 4; // Minimum heart size.
const uint_fast16_t MIN_HEART_RES    = 8;        // Minimum heart size.
const uint_fast16_t MAX_HEART_RES    = 128;      // Minimum heart size.

const uint_fast16_t MIN_RANDOM_POINTS = 20;      // Minimum number of random lines.
const uint_fast16_t MAX_RANDOM_POINTS = 100;     // Maximum number of random lines.

const size_t        MAX_LOG_STRING_SIZE = 100;   // Number of bytes in print buffer.
const size_t        PRINT_QUEUE_SIZE    = 8;     // Size of print queue.


/////////////////////////////////////////////////////////////////////////////////
// R U N T I M E   G L O B A L   V A R I A B L E S
/////////////////////////////////////////////////////////////////////////////////

// Current coordinates and angle.
volatile double CurrentX = 0.0;      // Current location of ball (X).
volatile double CurrentY = 0.0;      // Current location of ball (Y).
volatile double RadAngle = 0.0;      // Current angle of ball from the origin (radians).

// General runtime variables.
volatile int_fast32_t InOutSteps   = 0;      // Current # steps in/out is away from 0.
int_fast32_t          InOutStepsTo = 0;      // Number inout steps needed to reach target.
volatile int_fast32_t RotSteps     = 0;      // Current # steps rotary is away from 0.
int_fast32_t          RotStepsTo   = 0;      // Number rotary steps needed to reach target.
volatile bool         RotOn        = false;  // 'true' if rotary axis is enabled to move.
volatile bool         InOutOn      = false;  // 'true' if inout axis is enabled to move.
volatile uint_fast8_t DirInOut     = OUT;    // Current In/Out direction.
volatile uint_fast8_t DirRot       = CW;     // Current Rotary direction.
int_fast16_t          InOutDelay   = 400;    // ISR delay for In/Out motor (uSec).
int_fast16_t          RotDelay     = 400;    // ISR delay for Rotary motor (uSec).
uint_fast32_t         RandomSeed   = 0;      // RNG seed used at startup.
volatile uint_fast16_t MRPointCount = 0;     // Count of the number of points displayed.
double               RotSpeedFactor= 1.0;   // Multiplicative factor for rotational speed.
double               InOutSpeedFactor= 1.0; // Multiplicative factor for in/out speed.
volatile bool        Pausing      = false;  // 'true' when pausing do to pot setting.
volatile int_fast16_t InLimit      = 0;      // Inner limit where MotorRatios() will
                                             // change direction or finish.
volatile int_fast16_t OutLimit     = MAX_SCALE_I; // Outer limit where MotorRatios()
                                             // will change direction or finish.
         int_fast16_t SpeedDelay   = SPEED_DELAY_MAX_VAL;
                                             // Active speed delay value.
bool                  RemoteSpeedDelay = false; // 'true' if remotely setting speed.
int_fast16_t          Brightness   = 0;      // Active LED brightness value (0 - 255).
bool                  RemoteBrightness = false;   // 'true' if remotely setting brightness.
bool                  RemotePause  = false;  // 'true' if pausing motionremotely.
bool                  AbortShape   = false;  // 'true' if aborting ghe current shape.
uint_fast16_t         ShapeIteration = 0;    // Iteration counter for random shape generation.
bool                  RandomSeedChanged = false;  // 'true' when random seed has been change.

// Create our stepper state machines.
const float STEPPER_FREQUENCY = 1000000.0;  // Frequency for stepper state machines.
STStepper RotStepper(  DIR_ROT_PIN,   STEP_ROT_PIN,   STEPPER_FREQUENCY);
STStepper InOutStepper(DIR_INOUT_PIN, STEP_INOUT_PIN, STEPPER_FREQUENCY);


/////////////////////////////////////////////////////////////////////////////////
// F O R W A R D   D E C L A R A T I O N S
/////////////////////////////////////////////////////////////////////////////////
void    MotorRatios(double ratio, bool multiplePoints,
                 int_fast16_t inLimit = 0, int_fast16_t outLimit = MAX_SCALE_I);
int64_t RotaryServoIsr(__unused alarm_id_t id, __unused void *user_data);
int64_t InOutServoIsr(__unused alarm_id_t id, __unused void *user_data);


/////////////////////////////////////////////////////////////////////////////////
// S U P P O R T   S T R U C T U R E S   A N D   C L A S S E S
/////////////////////////////////////////////////////////////////////////////////

char logbuf[100][50];
uint_fast16_t logbufcount = 0;
/////////////////////////////////////////////////////////////////////////////////
// S U P P O R T   F U N C T I O N S
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// HandleRemoteCommandsTask()
//
// Checks for and handles remote commands from the serial port.
//
// Valid commands can be either upper or lower case, and include:
//  'F' (FASTER)      Increase the speed.
//  'S' (SLOWER)      Decreases the speed.
//  'Q'               Restores speed control to the local speed pot.
//  'B' (BRIGHTER)    Increases the LED brightness.
//  'D' (DARKER)      Decreases the LED brightness.
//  'L'               Restores LED control to the local LED pot.
//  'P' (PAUSE)       Pauses motion.
//  'U' (UNPAUSE)     Unpauses motion.
//  'R 'newSeed'
//      (RANDOM SEED) Re-seeds the random number generator with the value of
//                    'newSeed', homes the axes and clears the board.
//  'G' (GET)         Get the last random seed.
//  'N' (NEXT)        Aborts the current shape and starts the next one.
//  'K' (KEEP ALIVE)  Kicks the watchdog.  If no messages are received from the
//                    serial port after REMOTE_TIMEOUT_MS milliseconds, then all
//                    remote settings get cleared and local control is restored.
/////////////////////////////////////////////////////////////////////////////////
void HandleRemoteCommandsTask(__unused void *param)
{
    // Keep track of the last time we received a command.
    static uint_fast32_t lastMessageMs = millis();

    // Setup a buffer to hold any received commands.
    const uint_fast16_t BUFLEN = 80;
    char buf[BUFLEN];
    const char *HELLO_STRING = "Hello Sand Table!";

    // Delay before starting the task to give the print task time to start up.
    vTaskDelay(pdMS_TO_TICKS(3000));

    // This is a task, so we loop forever.
    while (1)
    {
        // See if it's been too long since our last serial message.
        // If so, then undo any remote controls that may be in effect.
        if (millis() - lastMessageMs > REMOTE_TIMEOUT_MS)
        {
            RemoteSpeedDelay  = false;
            RemoteBrightness  = false;
            RemotePause       = false;
            RandomSeedChanged = false;
            AbortShape        = false;
        }

        // See if we got any messages.
        if (Serial.available() > 0)
        {
            // Got a message.  Update our timeout base.
            lastMessageMs = millis();

            // Handle the remote command.
            int_fast16_t numBytesRead = Serial.readBytesUntil('\n', buf, BUFLEN - 1);
            // Remove any trailing junk from the input buffer.
            do
            {
                buf[numBytesRead--] = '\0';
            } while (isspace(buf[numBytesRead]) && (numBytesRead >= 0));
    if (logbufcount < 100)
    {
            strncpy(logbuf[logbufcount++], buf, 50);
    }
            switch (toupper(buf[0]))
            {
                // Increase speed.
                case 'F':
                    SpeedDelay -= 40;
                    RemoteSpeedDelay = true;
                break;
                // Decrease speed.
                case 'S':
                    SpeedDelay += 40;
                    RemoteSpeedDelay = true;
                break;
                // Restore local control of speed;
                case 'Q':
                    RemoteSpeedDelay = false;
                break;
                // Increase brightness.
                case 'B':
                    Brightness += 2;
                    RemoteBrightness = true;
                break;
                // Decrease brightness.
                case 'D':
                    Brightness -= 2;
                    RemoteBrightness = true;
                break;
                // Restore local control of brightness.
                case 'L':
                    RemoteBrightness = false;
                break;
                // Pause motion.
                case 'P':
                    RemotePause = true;
                break;
                // Unpause motion.
                case 'U':
                    RemotePause = false;
                break;
                // Re-start with a new random seed.
                case 'R':
                    // Read the new seed value.
                    char *endptr;   // Unused, but needed for strtoul().
                    RandomSeed = strtoul(buf + 1, &endptr, 10);
                    // Reset the random  number generator with the new seed value.
                    randomSeed(RandomSeed);
                    // Let everyone know we changed the random seed.
                    RandomSeedChanged = true;
                    // Abort the current shape since we want the new random value to
                    // take effect at the beginning of a shape.
                    AbortShape = true;
                    [[fallthrough]];
                // Get the (possibly) new random seed.
                // Note that the 'R' case falls through to this case.
                case 'G':
                    LOG_F(LOG_ALWAYS, "%d\n", RandomSeed);
                break;
                // Next shape - Abort the current shape and start the next.
                case 'N':
                    AbortShape = true;
                break;
                // Keep alive - do nothing.
                case 'K':
                break;
                // Hello - simply replies back with a canned message.
                case 'H':
                    if (!strcmp(buf, HELLO_STRING))
                    {
                        LOG_F(LOG_ALWAYS, "Hello Remote!\n");
                    }
                break;
    case 'Z':
    for (uint_fast16_t i = 0; i < logbufcount; i++)
    {
        Serial.printf("%2d - %s\n", i, logbuf[i]);
    }
    logbufcount = 0;
    break;
                // Default - Not anything we know about.  Just ignore it.
                default:
                break;
            }
            // Make sure that our speed and brightness values remain valid.
            SpeedDelay = constrain(SpeedDelay, SPEED_DELAY_MIN_VAL, SPEED_DELAY_MAX_VAL);
            Brightness = constrain(Brightness, BRIGHTNESS_MIN_VAL, BRIGHTNESS_MAX_VAL);
        }
        // Waste some time.
        vTaskDelay(pdMS_TO_TICKS(5));
    } // End while (1)  Tasks never return.
} // End HandleRemoteCommandsTask().




/////////////////////////////////////////////////////////////////////////////////
// ReadAPot()
//
// Returns a shifted value read from the specified potentiometer.
//
// Arguments:
// - pot : The pin number of the pot to be read.
//
// Returns:
//   Always returns the value of the specified analog input shifted right by
//   ANALOG_SHIFT_FACTOR bits.  This is done to work around the 2350 analog in
//   noise problem.
//
// Note:
//   Due to noise problems with the 2350 ADC, we reduce our ADC resolution by
//   4 bits (from 12 bits to 8 bits).  This is done by shifting the (noisy) 4
//   least significant bits out, leaving only the 8  most significant bits,
//   which is fine for this system.  A proper fix would require hardware
//   modification, and is not worth the effort in this case.
/////////////////////////////////////////////////////////////////////////////////
inline uint_fast16_t ReadAPot(int pot)
{
    return analogRead(pot) >> ANALOG_SHIFT_FACTOR;
} // End ReadAPot().


/////////////////////////////////////////////////////////////////////////////////
// UpdateLeds()
//
// Update the brightness of the LEDs.  If remote LED control is active, then
// just set the brightness.  Otherwise, read the brightness pot and set the LED
// brightness accordingly.
/////////////////////////////////////////////////////////////////////////////////
void UpdateLeds()
{
    // Only read left BRIGHTNESS pot if we're not being controlled remotely.
    if (!RemoteBrightness)
    {
        // The pot on the left is for the LEDs.  Read it now.
        uint_fast16_t brightnessKnobVal = ReadAPot(BRIGHTNESS_POT_PIN);
        Brightness = map(brightnessKnobVal, KNOB_MIN_VAL, KNOB_MAX_VAL,
                         BRIGHTNESS_MIN_VAL, BRIGHTNESS_MAX_VAL);
    }
    analogWrite(LIGHTS_PIN, Brightness);
} // End UpdateLeds().


/////////////////////////////////////////////////////////////////////////////////
// UpdateSpeeds()
//
// Update the servo speeds.  If remote speed control is active, then just set
// the speeds.  Otherwise, read the speed pot and set the speed delay values
// accordingly.
//
// If the speed pot is detected as being near zero, a pause state is entered.
// In this state, the axes are prohibited from moving.  This state remains in
// effect until the speed pot is no longer set to zero.
/////////////////////////////////////////////////////////////////////////////////
void UpdateSpeeds()
{
    // Only read the right SPEED pot if we're not being controlled remotely.
    if (!RemoteSpeedDelay)
    {
        // Pot on the right for drawing speed or delay.  Read it now.
        uint_fast16_t speedKnobVal = ReadAPot(SPEED_POT_PIN);
        SpeedDelay = map(speedKnobVal, KNOB_MIN_VAL, KNOB_MAX_VAL,
                         SPEED_DELAY_MAX_VAL,  SPEED_DELAY_MIN_VAL);
    }
    // Set the values atomically.
    noInterrupts();
    // Check against a value slightly lower than the max speed delay.
    Pausing    = (SpeedDelay >= (SPEED_DELAY_MAX_VAL - 512 / MICROSTEPS)) || RemotePause;
    RotDelay   = SpeedDelay * RotSpeedFactor;
    InOutDelay = SpeedDelay * InOutSpeedFactor;
    interrupts();

    // Indicate if we're pausing.
    digitalWrite(PAUSE_LED_PIN, Pausing);
} // End UpdateSpeeds().


/////////////////////////////////////////////////////////////////////////////////
// ReadPotsTask()
//
// Updates the brightness and speed values by reading the respective pots and
// setting the corresponding values based on the pot values.  
/////////////////////////////////////////////////////////////////////////////////
void ReadPotsTask(__unused void *param)
{
    while (1)
    {
        // Update the LED brightness based on pot input.
        UpdateLeds();

        // Update the speeds based on pot input and check for pause.
        UpdateSpeeds();

        // Since we are running at the same priority as the idle task (0), we
        // don't want to starve it, so we yield here.  Since configIDLE_SHOULD_YIELD
        // is enabled, the idle task will yield to us also.  If we don't mind
        // starving the idle task, we can just set this task's priority to 1 and
        // remove this taskYIELD() call.
        taskYIELD();
    } // End while (1)  Tasks never returns.
} // End ReadPotsTask();



/////////////////////////////////////////////////////////////////////////////////
// SetSpeedFactors()
//
// Sets the in/out and rotation speed factors as specified.
//
// Arguments:
//   - rotFactor   : The rotational factor to apply.  May be modified if it
//                   exceeds limits.
//   - inOutFactor : The in/out factor to apply.  May be modified if it exceeds
//                   limits.
/////////////////////////////////////////////////////////////////////////////////
void SetSpeedFactors(double rotFactor, double inOutFactor)
{
    rotFactor   = constrain(rotFactor,   MIN_MOTOR_RATIO, MAX_MOTOR_RATIO);
    inOutFactor = constrain(inOutFactor, MIN_MOTOR_RATIO, MAX_MOTOR_RATIO);

    // Set the values atomically so they take effect simultaneously.
    noInterrupts();
    RotSpeedFactor   = rotFactor;
    InOutSpeedFactor = inOutFactor;
    interrupts();
} // End SetSpeedFactors().


/////////////////////////////////////////////////////////////////////////////////
// EndShape()
//
// This function is normally called at the end of each shape execution.  It
// normally just delays for 3 seconds then returns.  However, it may also aid
// in debugging if PAUSE_ON_DONE is 'true'.  When enabled via the PAUSE_ON_DONE
// macro, execution will stop at the end of every shape until the speed pot is
// set to zero then non-zero.
/////////////////////////////////////////////////////////////////////////////////
void EndShape()
{
    // Announce that the shape is done.
    LOG_F(LOG_INFO, "End Shape\n");

    // If PAUSE_ON_DONE is 'false', then this entire block will be optimized out.
    if (PAUSE_ON_DONE)
    {
        // Wait for speed pot to be set to pause (0).
        while (!Pausing)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Now wait for speed pot pause to be cleared.
        while (Pausing)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    // Delay a bit to allow time to set speed pot back to desired value.
    vTaskDelay(pdMS_TO_TICKS(3000));
} // End EndShape().


/////////////////////////////////////////////////////////////////////////////////
// EndSeries()
//
// This function is called at the end of each series execution.  It simply
// announces that the series is done.
/////////////////////////////////////////////////////////////////////////////////
void EndSeries()
{
    LOG_F(LOG_INFO, "End Series\n");
} // End EndSeries().






/////////////////////////////////////////////////////////////////////////////////
// PrintTask()
//
// Creates a queue to accept data destined for the Serial output, then
// initializes the SerialLog2350 class which is used to request log messages.
// Waits in an infinite loop for messages to send out the Serial port and sends
// them when any are received.
/////////////////////////////////////////////////////////////////////////////////
void PrintTask(__unused void *param)
{
//vTaskDelay(pdMS_TO_TICKS(3000));
    // Create our queue and buffer.
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


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void PathTask(__unused void *param)
{
    // Delay before starting the task to give the print task time to start up.
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
} // End PathTask().



/////////////////////////////////////////////////////////////////////////////////
// R E Q U I R E D   A R D U I N O   F U N C T I O N S
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// setup()
//
// This function initializes all the hardware for core 0.
/////////////////////////////////////////////////////////////////////////////////
void setup()
{
    // Initially set all I/O pins except those used by the PIO state machines as
    // inputs with pullup per MicroChip recommendation to avoid floating inputs.
    for(int i = 0; i < 23; i++)
    {
        if ((i != STEP_ROT_PIN) && (i != DIR_ROT_PIN) &&
            (i != STEP_INOUT_PIN) && (i != DIR_INOUT_PIN))
        {
            pinMode(i, INPUT_PULLUP);
        }
    }

    // We set the PWM frequency to 100 khz and set the ADC resolution to 12 bits
    // (4096) to help reduce LED flicker at low LED knob settings.
    analogWriteFreq(PWM_FREQ);
    analogReadResolution(ANALOG_RESOLUTION);

    // Define GPIO pins per our hardware setup.
    pinMode(LIGHTS_PIN, OUTPUT);
    analogWrite(LIGHTS_PIN, BRIGHTNESS_MIN_VAL);
    pinMode(EN_ROT_PIN, OUTPUT);
    pinMode(EN_INOUT_PIN, OUTPUT);
    pinMode(PAUSE_LED_PIN, OUTPUT);
    pinMode(BRIGHTNESS_POT_PIN, INPUT);
    pinMode(SPEED_POT_PIN, INPUT);
    // Initialize the home input pin as input with pullup.
    if (USE_HOME_SENSOR)
    {
        pinMode(ROT_HOME_PIN, INPUT_PULLUP);
    }

    // Initialize our serial port, wait for port to open, and issue a "Starting"
    // message.  
    Serial.begin(115200);
    delay(100);
    // We can't use LOG_F() yet since it gets initialized in the print task
    // which hasn't started yet.
    Serial.print("\r\nStarting\n");

    // Make sure we start with valid speed factors.
    RotSpeedFactor    = 1.0;
    InOutSpeedFactor  = 1.0;

#if 0 //%%%jmc
    // Home the axes, then display status information.
    Home();
    Display();
#endif
    // Create an alarm for the rotational and in/out servo ISRs.  Start in 1 second.
    add_alarm_in_us(1000000, RotaryServoIsr, NULL, false);
    add_alarm_in_us(1000000, InOutServoIsr,  NULL, false);

    // Seed the random number generator.
    RandomSeed = get_rand_32();
    randomSeed(RandomSeed);

    // Display the random number.  It may be used in the future to repeat an
    // interesting sequence.  We can't use LOG_F yet since it gets initialized in
    // the print task which hasn't started yet.
    Serial.printf("Seed = %ul\n", RandomSeed);

    // Initialize runtime variables to safe values.
    SpeedDelay        = SPEED_DELAY_MAX_VAL;
    RemoteSpeedDelay  = false;
    Brightness        = BRIGHTNESS_MIN_VAL;
    RemoteBrightness  = false;
    RemotePause       = false;
    AbortShape        = false;
    ShapeIteration    = 0;
    RandomSeedChanged = false;
    
    // Special code for use while debugging.  Hold startup until the speed pot
    // is no longer in the pausing state.  If something we did caused FreeRTOS
    // to go into the weeds, this will allow our USB connection to be seen by
    // the host computer and allow us to recover.
    while (ReadAPot(SPEED_POT_PIN) <= KNOB_MIN_VAL + 16) { /* Do nothing.*/ }

    // Create the task that handles pot updates.  It will run on core 1.
    TaskHandle_t ReadPotsHandle = NULL;
    xTaskCreate(ReadPotsTask, "ReadPots", 1024, NULL, 0, &ReadPotsHandle);
    vTaskCoreAffinitySet(ReadPotsHandle, 1 << 1);

    // Create the task that handles remote serial commands.  It will run on core 1.
    TaskHandle_t HandleRemoteCommandsHandle = NULL;
    xTaskCreate(HandleRemoteCommandsTask, "RmtCmds", 4096, NULL, 2, &HandleRemoteCommandsHandle);
    vTaskCoreAffinitySet(HandleRemoteCommandsHandle, 1 << 1);

    // Create the task that logs data to the serial port.  It will run on core 1.
    TaskHandle_t PrintHandle = NULL;
    xTaskCreate(PrintTask, "Print", 4096, NULL, 2, &PrintHandle);
    vTaskCoreAffinitySet(PrintHandle, 1 << 1);
    
    // Create the task to generatse motion.  It will run on core 0.
    TaskHandle_t PathHandle = NULL;
    xTaskCreate(PathTask, "Path", 8192, NULL, 2, &PathHandle);
    vTaskCoreAffinitySet(PathHandle, 1 << 0);
    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // !!!NOTE:  MUST NOT CALL vTaskStartScheduler() OR SYSTEM WILL CRASH!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
} // End setup().

/////////////////////////////////////////////////////////////////////////////////
// loop()
//
// This is the main system loop for core 0.  Does nothing due to FreeRTOS use.
//
// Note: loop() must exist in FreeRTOS system and runs at priority 4.  Here
//       we don't want to use it so we simply delay for a long period.
/////////////////////////////////////////////////////////////////////////////////
void loop()
{
static int i = 0;    
LOG_F(LOG_ALWAYS, "Testing ... %d\n", i++)    ;
    vTaskDelay(pdMS_TO_TICKS(5000));
} // End loop().

/////////////////////////////////////////////////////////////////////////////////
// I N T E R R U P T   S E R V I C E   R O U T I N E S   ( I S R s)
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// ISR(TIMER1_COMPA_vect)
//
// This is the ISR for the rotational motor.
/////////////////////////////////////////////////////////////////////////////////
int64_t RotaryServoIsr(__unused alarm_id_t id, __unused void *user_data)
{
    // inOutCompAccum - accumulates in/out error based on rotary movement.  For
    // every GEAR_RATIO number of rotary steps, in/out moves 1 step in a direction
    // depending on the mechanical setup.  inOutCompAccum accumulates the error,
    // which is then used to ganerate compensation steps for the in/out axis.
    static int_fast16_t inOutCompAccum = 0;

    // Only do something if rotation movement is enabled.
    if (RotOn && !Pausing)
    {
        // Step the rotary servo in the proper direction.
        RotStepper.Step(DirRot);

        // Keep track of (accumulate) the number of rotational steps based
        // direction of rotation.
        int_fast16_t offsetDir = (DirRot == ROT_CAUSING_IN) ? 1 : -1;
        inOutCompAccum += offsetDir;
        RotSteps += offsetDir;

        // Rotations cause in/out mechanical movement.  When the limit is
        // reached, we compensate by adding a step in the proper in/out
        // direction.
        if (abs(inOutCompAccum) >= GEAR_RATIO)
        {
            // Reset our compensation accumulator.
            inOutCompAccum = 0;
            if (InOutOn)
            {
                // The in/out motor is already moving.  Simply update the
                // InOutSteps counter to account for our compensation.
                InOutSteps -= offsetDir;
            }
            else
            {
                // The in/out motor is not moving.  Force our compensation
                // by actually stepping the in/out motor in the correct
                // direction.
                DirInOut = (DirRot == CW) ? IN : OUT;
                InOutStepper.Step(DirInOut);
            }
        }

        // Wrap-around conditions for rotational steps.
        if (RotSteps > ROT_TOTAL_STEPS)
        {
            RotSteps = 0;
        }
        else if (RotSteps < 0)
        {
            RotSteps = ROT_TOTAL_STEPS - 1;
        }

        // Update our (possibly new) rotational angle which is used in background
        // processing.
        RadAngle = ((double)RotSteps * PI_X_2) / (double)ROT_TOTAL_STEPS;

        // Complete the move if we've reached our target and are not using
        // multiple points.
        if (MRPointCount == 0)
        {
            if (RotSteps == RotStepsTo)
            {
                // Turn off the motor if it reaches the set point.
                RotOn = false;
            }
            if (InOutSteps == InOutStepsTo)
            {
                InOutOn = false;
            }
        }
    } // End if (RotOn && !Pausing)

    // In order to restart the alarm, we return a negative delay time.  This
    // causes the timer subsystem to reschedule the alarm this many microseconds
    // from the time this ISR started.
    return -RotDelay;
} // End ISR(TIMER1_COMPA_vect).


/////////////////////////////////////////////////////////////////////////////////
// ISR(TIMER1_COMPB_vect)
//
// This is the ISR for the in/out motor.
/////////////////////////////////////////////////////////////////////////////////
int64_t InOutServoIsr(__unused alarm_id_t id, __unused void *user_data)
{
    // Used to remember the last iteratioins in/out direction.
    static uint_fast8_t lastInOutDir = DirInOut;

    // Used to determine first time use of multiple points.
    static uint_fast16_t lastMRPoints = 0;

    // Only do something if in/out movement is enabled.
    if (InOutOn && !Pausing)
    {
        // Step the in/out servo in the proper direction.
        InOutStepper.Step(DirInOut);

        // Update in/out steps based on the direction.
        if (DirInOut == OUT)
        {
            InOutSteps++;
        }
        else
        {
            InOutSteps--;
        }

        // Handle normal case of not using multiple points first.
        if (MRPointCount == 0)
        {
            // Complete the move if we've reached our target and are not
            // using multiple points.
            if (InOutSteps == InOutStepsTo)
            {
                InOutOn = false;
            }
            lastMRPoints = false;
        }
        else  // MRPointCount != 0.
        {
            // First time using multiple points - remember our initial in/out
            // direction.
            if (!lastMRPoints)
            {
                lastInOutDir = DirInOut;
                lastMRPoints = true;
            }
            // Adjust the direction if limits are reached.
            if (InOutSteps > OutLimit)
            {
                DirInOut = IN;
            }
            else if (InOutSteps < InLimit)
            {
                DirInOut = OUT;
            }

            // If we're creating multiple points and the direction has changed,
            // deccrement the points count.
            if (lastInOutDir != DirInOut)
            {
                if (--MRPointCount == 0)
                {
                    RotOn   = false;
                    InOutOn = false;
                }
                // Remember the current in/out direction for next time.
                lastInOutDir = DirInOut;
            }
        } // End else (MRPointCount)
    } // End if (InOutOn && !Pausing)

    // In order to restart the alarm, we return a negative delay time.  This
    // causes the timer subsystem to reschedule the alarm this many microseconds
    // from the time this ISR started.
    return -InOutDelay;
} // End ISR(TIMER1_COMPB_vect).
