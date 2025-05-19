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
#include <limits.h>         // For UINT_MAX.
#include "SerialLog2350.h"  // For data logging macro (LOG_F).
#include "STStepper.pio.h"  // For STStepper class and stepper PIO state machine.


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
const uint_fast32_t HOME_ROT_OFFSET   = 347 * (ROT_TOTAL_STEPS / 1000);
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
bool                  Pausing      = false;  // 'true' when pausing do to pot setting.
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

/////////////////////////////////////////////////////////////////////////////////
// struct Coordinate
//
// Structure defining one cartesian (x, y) point.  Used with PlotShapeArray().
/////////////////////////////////////////////////////////////////////////////////
struct Coordinate
{
    Coordinate(int_least8_t px, int_least8_t py)
    {
        x = px;
        y = py;
    }
    int_least8_t x;   // X coordinate.
    int_least8_t y;   // Y coordinate.
}; // End Coordinate.


/////////////////////////////////////////////////////////////////////////////////
// struct PlotInfo
//
// This structure contains a pointer to a plot array, its size, and bool to
// select whether or not the plot may be rotated.  It is used in RandomPlot() to
// select a random plot array to display.
/////////////////////////////////////////////////////////////////////////////////
struct PlotInfo
{
    const Coordinate *m_Plot;    // Pointer to a plot array.
    uint_fast16_t     m_Size;    // Size of the array pointed to by m_Plot.
    bool              m_Rotate;  // True to allow rotation.
    const char       *m_pName;   // Name string.
}; // End PlotInfo.


/////////////////////////////////////////////////////////////////////////////////
// class ShapeInfo
//
// This class contains information pertaining to a particular random shape
// generator.  It is used to enforce minimum delays between consecutive
// executions of the same shape.  It does this by remembering the last time
// that the shape was executed, and comparing it to the current time.
/////////////////////////////////////////////////////////////////////////////////
class ShapeInfo
{
public:
    /////////////////////////////////////////////////////////////////////////////
    // Constructor
    //
    // This is the only valid way to create an instance.  The default constructor
    // and copy constructor are defined as private, so are not available for
    // external use.
    //
    // Arguments:
    //   - pShape : This is a pointer to the function that will generate a shape.
    //              The function has no arguments and returns nothing.
    //   - delay  : The number of execution cycles that must elapse between
    //              executions of *pShape().
    /////////////////////////////////////////////////////////////////////////////
    ShapeInfo(void (*pShape)(), uint_fast16_t delay)
    {
        // Initialize our data based on our arguments.
        m_pShape    = pShape;
        m_Delay     = delay;
        m_LastCycle = 0;
    } // End constructor.


    /////////////////////////////////////////////////////////////////////////////
    // MakeShape()
    //
    // This method enforces the delay between consecutive executions of pShape().
    //
    // Arguments:
    //   - cycle : This is the current cycle (iteration) of shape generation
    //             It is incremented by the caller each time a shape is
    //             generated.
    /////////////////////////////////////////////////////////////////////////////
    bool MakeShape(uint_fast16_t cycle)
    {
        // Assume we're going to fail.
        bool retval = false;

        // Have enough iterations passed since our last execution?
        if (cycle - m_LastCycle >= m_Delay)
        {
            // Yes, it's ok to make our shape.  Do it!
            (*m_pShape)();

            // Remember that we just executed.
            m_LastCycle = cycle;
            retval = true;
        }
        // Let the caller know if we executed or not.
        return retval;
    } // End MakeShape().


    /////////////////////////////////////////////////////////////////////////////
    // Reset()
    //
    // This method resets the last cycle value.  It is mainly used when a new
    // random seed is set so that a previously generated sequence can be
    // identically repeated.
    /////////////////////////////////////////////////////////////////////////////
    void Reset()
    {
        m_LastCycle = 0;
    } // End Reset().


private:
    // Private methods so the user can't call them.
    ShapeInfo();
    ShapeInfo &operator=(ShapeInfo &);

    void         (*m_pShape)();  // Pointer to function to execute if not too soon.
    uint_fast16_t m_Delay;       // Number of iterations between consecutive executions.
    uint_fast16_t m_LastCycle;   // Iteration that we last executed.
}; // End class ShapeInfo.


/////////////////////////////////////////////////////////////////////////////////
// S H A P E   A R R A Y S
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// MazePlot[]
//
// Array of PlotPoints to plot a maze.
/////////////////////////////////////////////////////////////////////////////////
const Coordinate MazePlot[] =
{   {73, -68}, {73, -60}, {60, -60}, {60, -80}, {59, -81}, {55, -84}, {50, -87},
    {46, -89}, {46, -87}, {33, -87}, {33, -95}, {30, -96}, {26, -97}, {21, -98},
    {16, -99}, {11, -100}, {6, -100}, {6, -87}, {20, -87}, {20, -74}, {6, -74},
    {6, -60}, {33, -60}, {33, -74}, {46, -74}, {46, -34}, {33, -34}, {33, -47},
    {6, -47}, {6, -34}, {20, -34}, {20, -20}, {6, -20}, {6, -7}, {33, -7},
    {33, -20}, {46, -20}, {46, -7}, {73, -7}, {73, -20}, {60, -20}, {60, -47},
    {73, -47}, {73, -34}, {86, -34}, {86, -47}, {88, -47}, {90, -44}, {92, -39},
    {93, -35}, {95, -30}, {96, -25}, {97, -20}, {86, -20}, {86, -7}, {99, -7},
    {99, -5}, {99, 0}, {99, 5}, {99, 10}, {98, 15}, {97, 20}, {86, 20}, {86, 6},
    {60, 6}, {60, 20}, {73, 20}, {73, 33}, {60, 33}, {60, 46}, {86, 46}, {86, 33},
    {94, 33}, {92, 37}, {90, 42}, {88, 46}, {86, 50}, {83, 55}, {80, 59}, {77, 63},
    {74, 66}, {71, 70}, {67, 73}, {63, 76}, {60, 80}, {60, 73}, {67, 73}, {69, 71},
    {73, 67}, {73, 60}, {33, 60}, {33, 73}, {46, 73}, {46, 88}, {42, 90}, {37, 92},
    {33, 94}, {33, 86}, {20, 86}, {20, 97}, {16, 98}, {11, 99}, {6, 99}, {6, 73},
    {20, 73}, {20, 60}, {6, 60}, {6, 33}, {20, 33}, {20, 46}, {46, 46}, {46, 33},
    {33, 33}, {33, 20}, {46, 20}, {46, 6}, {20, 6}, {20, 20}, {6, 20}, {6, 6},
    {-7, 6}, {-7, 20}, {-20, 20}, {-20, 6}, {-47, 6}, {-47, 20}, {-34, 20},
    {-34, 33}, {-47, 33}, {-47, 46}, {-20, 46}, {-20, 33}, {-7, 33}, {-7, 60},
    {-20, 60}, {-20, 73}, {-7, 73}, {-7, 99}, {-11, 99}, {-16, 98}, {-20, 97},
    {-20, 86}, {-34, 86}, {-34, 94}, {-38, 92}, {-43, 90}, {-47, 88}, {-47, 73},
    {-34, 73}, {-34, 60}, {-74, 60}, {-74, 67}, {-72, 69}, {-68, 73}, {-60, 73},
    {-60, 80}, {-64, 77}, {-68, 73}, {-71, 70}, {-75, 66}, {-78, 63}, {-81, 59},
    {-84, 55}, {-86, 51}, {-89, 46}, {-91, 42}, {-93, 37}, {-95, 33}, {-87, 33},
    {-87, 46}, {-60, 46}, {-60, 33}, {-74, 33}, {-74, 20}, {-60, 20}, {-60, 6},
    {-87, 6}, {-87, 20}, {-98, 20}, {-99, 17}, {-100, 12}, {-100, 8}, {-100, 3},
    {-100, -2}, {-100, -7}, {-87, -7}, {-87, -20}, {-98, -20}, {-98, -24},
    {-96, -29}, {-95, -34}, {-93, -38}, {-91, -43}, {-89, -47}, {-87, -47},
    {-87, -34}, {-74, -34}, {-74, -47}, {-60, -47}, {-60, -20}, {-74, -20},
    {-74, -7}, {-47, -7}, {-47, -20}, {-34, -20}, {-34, -7}, {-7, -7}, {-7, -20},
    {-20, -20}, {-20, -34}, {-7, -34}, {-7, -47}, {-34, -47}, {-34, -34},
    {-47, -34}, {-47, -74}, {-34, -74}, {-34, -60}, {-7, -60}, {-7, -74},
    {-20, -74}, {-20, -87}, {-7, -87}, {-7, -100}, {-10, -100}, {-15, -99},
    {-20, -99}, {-24, -98}, {-29, -96}, {-34, -95}, {-34, -87}, {-47, -87},
    {-47, -89}, {-48, -88}, {-52, -86}, {-57, -83}, {-60, -80}, {-60, -60},
    {-74, -60}, {-74, -68}, {-71, -71}
}; // End MazePlot.


/////////////////////////////////////////////////////////////////////////////////
// JMCPlot[]
//
// Array of PlotPoints to plot a JMC script text.
/////////////////////////////////////////////////////////////////////////////////
const Coordinate JMCPlot[] =
{   {-94, -34}, {-69, -25}, {-61, -25}, {-58, -22}, {-53, -21}, {-58, -8}, {-61, 2},
    {-62, 16}, {-63, 25}, {-63, 39}, {-62, 53}, {-60, 63}, {-58, 66}, {-56, 68},
    {-50, 68}, {-49, 65}, {-47, 58}, {-47, -4}, {-48, -15}, {-48, -26}, {-49, -35},
    {-48, -45}, {-49, -55}, {-50, -63}, {-51, -67}, {-53, -69}, {-59, -69},
    {-60, -68}, {-62, -63}, {-62, -46}, {-61, -35}, {-59, -28}, {-58, -22},
    {-54, -21}, {-51, -22}, {-48, -23}, {-46, -24}, {-38, -24}, {-35, -25},
    {-21, -25}, {-17, -22}, {-17, -8}, {-16, -2}, {-16, 12}, {-15, 19}, {-15, 39},
    {-14, 46}, {-15, 53}, {-16, 59}, {-17, 66}, {-20, 69}, {-23, 66}, {-25, 62},
    {-28, 56}, {-25, 62}, {-23, 66}, {-20, 69}, {-17, 66}, {-16, 59}, {-15, 53},
    {-14, 46}, {-15, 39}, {-15, 19}, {-16, 12}, {-16, -2}, {-17, -8}, {-17, -22},
    {-17, -8}, {-16, -2}, {-16, 12}, {-15, 19}, {-15, 39}, {-14, 46}, {-13, 56},
    {-11, 64}, {-7, 69}, {-3, 69}, {-1, 65}, {0, 60}, {1, 55}, {3, 46}, {2, 39},
    {2, 19}, {1, 12}, {1, -2}, {0, -8}, {0, -22}, {0, -8}, {1, -2}, {1, 12},
    {2, 19}, {2, 39}, {3, 46}, {4, 57}, {6, 66}, {8, 69}, {10, 71}, {15, 71},
    {16, 67}, {17, 63}, {18, 58}, {19, 54}, {18, 46}, {18, -12}, {17, -19},
    {21, -22}, {24, -25}, {40, -25}, {42, -23}, {46, -19}, {44, -14}, {43, -11},
    {42, -5}, {41, 2}, {41, 23}, {42, 32}, {42, 40}, {44, 48}, {46, 56}, {49, 62},
    {52, 66}, {56, 68}, {59, 67}, {61, 60}, {63, 51}, {64, 42}, {63, 51}, {61, 60},
    {59, 67}, {56, 68}, {52, 66}, {49, 62}, {46, 56}, {44, 48}, {42, 40}, {42, 32},
    {41, 23}, {41, 2}, {42, -5}, {44, -14}, {46, -19}, {49, -23}, {53, -25},
    {71, -25}, {96, -34}

}; // End JMCPlot.


/////////////////////////////////////////////////////////////////////////////////
// S U P P O R T   F U N C T I O N S
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// HandleRemoteCommands()
//
// Checks for and handles remote commands froom the serial port.
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
void HandleRemoteCommands()
{
    // Keep track of the last time we received a command.
    static uint_fast32_t lastMessageMs = millis();

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
        // Setup a buffer to hold any received commands.
        const uint_fast16_t BUFLEN = 40;
        char buf[BUFLEN];
        uint_fast16_t len = 0;

        // Got a message.  Update our timeout base.
        lastMessageMs = millis();

        // Handle the remote command.
        buf[0] = Serial.read();
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
            // Start with a new random seed.
            case 'R':
                // Read the new seed value.
                len = Serial.readBytesUntil('\n', buf, BUFLEN - 1);
                buf[len] = '\0';
                char *endptr;   // Unused, but needed for strtoul().
                RandomSeed = strtoul(buf, &endptr, 10);
                // Reset the random  number generator with the new seed value.
                randomSeed(RandomSeed);
                // Let everyone know we changed the random seed.
                RandomSeedChanged = true;
                // Abort the current shape since we want the new random value to
                // take effect at the beginning of a shape.
                AbortShape = true;
                [[fallthrough]];
            // Get the last random seed.
            // Note that the 'R' case falls through to this case.
            case 'G':
                LOG_F(LOG_ALWAYS, "%ul\n", RandomSeed);
                break;
            // Next shape - Abort the current shape and start the next.
            case 'N':
                AbortShape = true;
            break;
            // Keep alive - do nothing.
            case 'K':
            break;
            // Default - Not anything we know about.  Just ignore it.
            default:
            break;
        }
        // Make sure that our speed and brightness values remain valid.
        SpeedDelay = constrain(SpeedDelay, SPEED_DELAY_MIN_VAL, SPEED_DELAY_MAX_VAL);
        Brightness = constrain(Brightness, BRIGHTNESS_MIN_VAL, BRIGHTNESS_MAX_VAL);
    }
} // End HandleRemoteCommands().


/////////////////////////////////////////////////////////////////////////////////
// RandomBool()
//
// Ret;urns a random boolean value ('true' or 'false').
/////////////////////////////////////////////////////////////////////////////////
bool RandomBool() { return (bool)random(0, 2); }


/////////////////////////////////////////////////////////////////////////////////
// RandomFloat()
//
// Returns a random double value between minVal and maxVal with 2 digits precision.
//
// Arguments:
//   - minVal : Minimum value of the returned random double.
//   - maxVal : Maximum vlaue of the returned random double.
//
// Returns:
//   Returns a random double between the specified limits.
/////////////////////////////////////////////////////////////////////////////////
double RandomFloat(double minVal, double maxVal)
{
    return (double)random((long)(minVal * FLOAT_PRECISION),
                           (long)(maxVal * FLOAT_PRECISION) + 1) / FLOAT_PRECISION;
} // End RandomFloat().


/////////////////////////////////////////////////////////////////////////////////
// DtoR()
//
// Converts degrees to radians.
//
// Arguments:
//   - degrees : The degree value to be converted to radians.
//
// Returns:
//   Returns the value of the degree argument converted to radians.
/////////////////////////////////////////////////////////////////////////////////
double DtoR(double degrees)
{
    return degrees * PI_X_2 / 360.0;
} // End DtoR().


/////////////////////////////////////////////////////////////////////////////////
// RtoD()
//
// Converts radians to degrees.
//
// Arguments:
//   - rads : The radian value to be converted to degrees.
//
// Returns:
//   Returns the value of the rads argument converted to degrees.
/////////////////////////////////////////////////////////////////////////////////
double RtoD(double rads)
{
    return rads * 360.0 / PI_X_2;
} // End RtoD().


/////////////////////////////////////////////////////////////////////////////////
// GCD()
//
// Returns the greatest common divisor (gcd) of the 2 inputs.  Uses the
// subtraction-based Euclid's algorithm.
//
// Arguments:
//  a - The first unsigned 16-bit value.
//  b - The second unsigned 16-bit value.
//
// Returns:
//  Returns the GCD of the two input values.
/////////////////////////////////////////////////////////////////////////////////
uint_fast16_t GCD(uint_fast16_t a, uint_fast16_t b)
{
    while (a != b)
    {
        if (a > b)
        {
            a -= b;
        }
        else
        {
            b -= a;
        }
    }
    return a;
} // End GCD().


/////////////////////////////////////////////////////////////////////////////////
// LCM()
//
// Returns the least common multiple (lcm) of its 2 uint_fast16_t arguments.
//
// Arguments:
//  a - The first unsigned 16-bit value.
//  b - The second unsigned 16-bit value.
//
// Returns:
//   Always returns the lcm of 'a' and 'b'.
/////////////////////////////////////////////////////////////////////////////////
uint_fast16_t LCM(uint_fast16_t a, uint_fast16_t b)
{
    return (a * b) / GCD(a, b);
} // End LCM().


/////////////////////////////////////////////////////////////////////////////////
// Reduce()
//
// Returns the input values reduced by their greatest common divisor (gcd).
//
// Arguments:
//  a - The first unsigned 16-bit value.
//  b - the second unsigned 16-bit value.
//
// Returns:
//  a - The (possibly) reduced 'a' value.
//  b - The (possibly) reduced 'b' value.
/////////////////////////////////////////////////////////////////////////////////
void Reduce(uint_fast16_t &a, uint_fast16_t &b)
{
    uint_fast16_t gcd = GCD(a, b);
    a /= gcd;
    b /= gcd;
} // End Reduce().


/////////////////////////////////////////////////////////////////////////////////
// CalculateXY()
//
// Calculates the current X/Y coordinates based on the current values of
// InOutSteps and RadAngle.  Sets CurrentX and CurrentY accordingly.
/////////////////////////////////////////////////////////////////////////////////
void CalculateXY()
{
    double r = (double)(InOutSteps * MAX_SCALE_I) / (double)INOUT_TOTAL_STEPS;
    CurrentX  = r * cos(RadAngle);
    CurrentY  = r * sin(RadAngle);
} // End CalculateXY().


/////////////////////////////////////////////////////////////////////////////////
// ForceInOut()
//
// This function steps the in/out axis IN or OUT without coordinating with the
// rest of the system.  It is mainly used during homing to establish the full
// in or out position and should not normally be called by shape generation code.
//
// Arguments:
//   - steps     : The number of motor steps to go IN or OUT.  We limit this to
//                 values between 0 and INOUT_TOTAL_STEPS + 10 in order to
//                 protect the in/out hardware.
//   - direction : The direction (IN or OUT) to move the axis.
//   - delay     : The delay, in microseconds, to add between motor step toggles.
//                 Higher values produce slower speeds.  Good values range from
//                 100 to 1000.
/////////////////////////////////////////////////////////////////////////////////
void ForceInOut(int_fast32_t steps, uint_fast8_t direction, uint_fast16_t delay)
{
    // Limit our arguments.
    steps = min(steps, INOUT_TOTAL_STEPS + 10);
    delay = constrain(delay, MIN_FORCE_DELAY, MAX_FORCE_DELAY);

    // Enable the stepper motor.
    EnableInOut();

    // Set the in/out direction.
    DirInOut = direction;

    // Move the in/out axis as specified.
    for (int_fast32_t x = 0; x < steps; x++)
    {
        InOutStepper.Step(DirInOut);
        delayMicroseconds(delay);
        UpdateLeds();
    }
} // End ForceInOut().


/////////////////////////////////////////////////////////////////////////////////
// ForceRot()
//
// This function steps the rotary axis CW or CCW without coordinating with the
// rest of the system.  It is mainly used during homing to search for the rotary
// home sensor and should not normally be called by shape generation code.
//
//
// Arguments:
//   - steps     : The number of motor steps to go CW or CCW.
//   - direction : The direction (CW or CCW) to move the axis.
//   - delay     : The delay, in microseconds, to add between motor step toggles.
//                 Higher values produce slower speeds.  Good values range from
//                 100 to 1000.
/////////////////////////////////////////////////////////////////////////////////
void ForceRot(int_fast32_t steps, uint_fast8_t direction, uint_fast16_t delay)
{
    // Limit our arguments.
    delay = constrain(delay, MIN_FORCE_DELAY, MAX_FORCE_DELAY);

    // Enable the stepper motor.
    EnableRot();

    // Set direction.
    DirRot = direction;

    // Move the rotation axis as specified.
    for (int_fast32_t x = 0; x < steps; x++)
    {
        RotStepper.Step(DirRot);
        delayMicroseconds(delay);
        UpdateLeds();
    }
} // End ForceRot().


/////////////////////////////////////////////////////////////////////////////////
// Home()
//
// Home both the in/out and rotary axes.  The home function is normally done
// at startup in order to initialize the starting position of the axes to
// (0, 0) coordinates.  It does not coordinate its moves with the rest of the
// system in that the axes movements are not registered in CurrentX and CurrentY.
// Upon completion, the pointer will be positioned to (0, 0), and all position
// related variables will be updated to be consistent with the (0, 0) position.
/////////////////////////////////////////////////////////////////////////////////
void Home()
{
    LOG_F(LOG_INFO, "Home()\n");

    // Only handle the home sensor if it exists.  The compiler will optimize the
    // following code out if USE_HOME_SENSOR is 'false'.
    if (USE_HOME_SENSOR)
    {
        // Make sure the motor driver ISRs don't interfere with us.
        DisableRot();
        DisableInOut();
        RotOn = false;
        InOutOn = false;

        // Move the inout axis first, all the way out.
        DirInOut = OUT;
        ForceInOut(INOUT_TOTAL_STEPS, DirInOut, MIN_FORCE_DELAY);

        // Disable inout to eliminate clicking on CW move.
        DisableInOut();

        // If already on the home position, rotate  CCW till we're past it.
        DirRot = CCW;
        while (IsRotHome())
        {
             ForceRot(1, DirRot, MAX_FORCE_DELAY);
        }

        // Rotate CW until the rotary home switch is detected.
        DirRot = CW;
        while (!IsRotHome())
        {
             ForceRot(1, DirRot, MAX_FORCE_DELAY / 4);
        }

        // Disable inout to eliminate clicking on CW move.
        DisableInOut();
        // Apply the homing rotational offset (if any).
        DirRot = CCW;
        ForceRot(HOME_ROT_OFFSET, DirRot, MIN_FORCE_DELAY * 3);
    } // End USE_HOME_SENSOR.

    // Retract the inout axis all the way (to zero).
    DirInOut = IN;
    ForceInOut(INOUT_TOTAL_STEPS, DirInOut, MIN_FORCE_DELAY * 3);

    // We are now at position (0, 0).  Update our position related variables.
    CurrentX     = 0.0;
    CurrentY     = 0.0;
    InOutSteps   = 0;
    InOutStepsTo = 0;
    RotSteps     = 0;
    RotStepsTo   = 0;
    RotOn        = false;
    InOutOn      = false;
    MRPointCount = 0;
    InLimit      = 0;
    OutLimit     = MAX_SCALE_I;

    // Enable both motors.  This should be done last.
    EnableInOut();
    EnableRot();
} // End Home().


/////////////////////////////////////////////////////////////////////////////////
// ReverseKinematics()
//
// This function calculates the necessary values of RotStepsTo and InOutStepsTo
// given the (X, Y) coordinates of the target position (i.e. it performs a
// reverse kinematics operation).  RotStepsTo and InOutStepsTo are in motor
// counts from zero, and are used by the motor driver ISRs to position the
// axes to the specified target location.
//
// Arguments:
//   - newX : This is the target X coordinate position.
//   - newY : This is the target Y coordinate position.
/////////////////////////////////////////////////////////////////////////////////
void ReverseKinematics(double newX, double newY)
{
    // Calculate radial distance from the origin to the target point.
    double rTo = hypot(newX, newY);

    // Calculate angle from the origin using atan2().
    double angleTo = atan2(newY, newX);

    // atan2() returns the angle in radians in the range -pi to pi.  Convert
    // negative angle to positive.
    if (angleTo < 0.0)
    {
        angleTo = PI_X_2 + angleTo;
    }

    // Calculate the rotation steps and limit as needed.
    RotStepsTo = round((angleTo * ROT_TOTAL_STEPS) / PI_X_2);
    if (RotStepsTo >= ROT_TOTAL_STEPS)
    {
        RotStepsTo -= ROT_TOTAL_STEPS;
    }

    // Calculate the in/out steps and limit as needed.
    InOutStepsTo = round((rTo * INOUT_TOTAL_STEPS) / MAX_SCALE_F);
    InOutStepsTo = constrain(InOutStepsTo, 0, INOUT_TOTAL_STEPS);

    // Determine whether to go IN or OUT.
    bool bInOutON = (InOutSteps != InOutStepsTo);
    if (bInOutON)
    {
        if (InOutSteps > InOutStepsTo)
        {
            DirInOut = IN;  // Go in.
        }
        else
        {
            DirInOut = OUT;  // Go out.
        }
    }

    // Determine whether to go CW or CCW.
    bool bRotON = (RotSteps != RotStepsTo);
    if (bRotON)
    {
        if (((RotStepsTo > RotSteps) &&
             (RotStepsTo - RotSteps < ROT_TOTAL_STEPS / 2)) ||
             ((RotStepsTo < RotSteps) && (RotSteps - RotStepsTo > ROT_TOTAL_STEPS / 2)))
        {
            DirRot = CCW;
        }
        else
        {
            DirRot = CW;
        }
    }

    // Start the move atomically.
    noInterrupts();
    InOutOn = bInOutON;
    RotOn   = bRotON;
    interrupts();
} // End ReverseKinematics().


/////////////////////////////////////////////////////////////////////////////////
// EnableInOut(), DisableInOut(), EnableRot(), DisableRot()
//
// These inline functions are used to enable and disable the individual axes.
// They take no arguments and return nothing.
/////////////////////////////////////////////////////////////////////////////////
inline void EnableInOut()  { digitalWrite(EN_INOUT_PIN, LOW);  }
inline void DisableInOut() { digitalWrite(EN_INOUT_PIN, HIGH); }
inline void EnableRot()    { digitalWrite(EN_ROT_PIN,   LOW);  }
inline void DisableRot()   { digitalWrite(EN_ROT_PIN,   HIGH); }


/////////////////////////////////////////////////////////////////////////////////
// ExtendInOut()
//
// Moves to a position where the in/out arm is fully extended while keeping
// the angle from the origin unchanged.
/////////////////////////////////////////////////////////////////////////////////
void ExtendInOut()
{
    GotoXY(MAX_SCALE_F * cos(RadAngle), MAX_SCALE_F * sin(RadAngle));
} // End ExtendInOut().


/////////////////////////////////////////////////////////////////////////////////
// RotateToAngle()
//
// Rotates the rotary axis to a specified angle from the origin leaving the
// in/out arm position unchanged.
//
// Arguments:
//   - angle : The target angle from the origin to which the the rotary axis
//             will move.  This move is coordinated with the system.
/////////////////////////////////////////////////////////////////////////////////
void RotateToAngle(double angle)
{
    // atan2() returns the angle in radians in the range -pi to pi.  Convert
    // negative angle to positive.
    angle += (angle < 0.0) ? PI_X_2 : 0.0;

    // Calculate the rotation steps and limit as needed.
    RotStepsTo = (int_fast16_t)round(((angle * (double)ROT_TOTAL_STEPS) / PI_X_2));
    if (RotStepsTo >= ROT_TOTAL_STEPS)
    {
        RotStepsTo -= ROT_TOTAL_STEPS;
    }

    // Don't do any mnore if we're already at the target rotation.
    if (RotStepsTo != RotSteps)
    {
        // Set the direction and start the move.
        DirRot = (((RotStepsTo > RotSteps) &&
                   (RotStepsTo - RotSteps < ROT_TOTAL_STEPS / 2)) ||
                  ((RotStepsTo < RotSteps) && (RotSteps - RotStepsTo > ROT_TOTAL_STEPS / 2)))
                   ? CCW : CW;
        RotOn = true;

        // Wait for the move to complete.
        while(RotOn)
        {
            CalculateXY();
            ReadPots();
        }
        CalculateXY();
    }
} // End RotateToAngle().


/////////////////////////////////////////////////////////////////////////////////
// IsRotHome()
//
// Returns an indication of the home input switch state.  Returns 'true' on home
// switch active.  Returns false otherwise.
/////////////////////////////////////////////////////////////////////////////////
inline bool IsRotHome()  { return !digitalRead(ROT_HOME_PIN); }


/////////////////////////////////////////////////////////////////////////////////
// MoveTo()
//
// Move to a specified point in an uncoordinated manner (without interpolation).
//
// Arguments:
//   x, y : The cartesian coordinates of the target position.
/////////////////////////////////////////////////////////////////////////////////
void MoveTo(double x, double y)
{
    // Calculate the target in/out and rotary values and start moving.
    ReverseKinematics(x, y);

    // Wait until the target is reached.
    while (RotOn || InOutOn)
    {
        ReadPots();
    }
} // End MoveTo().


/////////////////////////////////////////////////////////////////////////////////
// GotoXY()
//
// Move to a specified point in a coordinated manner (with interpolation).
//
// Arguments:
//   targetX, targetY : The cartesian coordinates of the target position.
/////////////////////////////////////////////////////////////////////////////////
void GotoXY(double targetX, double targetY)
{
    // Make sure we stay within the bounds of the table.
    if ((sq(targetX) + sq(targetY)) > (MAX_SCALE_F * MAX_SCALE_F))
    {
        // They want to go beyond the limits of the table.  We compensate by
        // limiting the move to safe bounds.
        double angle = atan2(targetY, targetX);
        targetX = trunc(MAX_SCALE_F * cos(angle));
        targetY = trunc(MAX_SCALE_F * sin(angle));
    }
    // Calculate the difference between target and current positions.
    double dx = targetX - CurrentX;
    double dy = targetY - CurrentY;
    // Calculate the total distance to travel.
    double distance = hypot(dx, dy);
    // Calculate the number of steps required, and round.
    // Adjust stepsPerUnit according to your system.
    uint_fast16_t steps = (uint_fast16_t)round(distance * STEPS_PER_UNIT);
    // Exit right away if the move is too small.
    if (steps != 0)
    {
        // Initialize our intermediate position variables.
        double newX = CurrentX;
        double newY = CurrentY;

        // Calculate the increments per step.
        double incrementX = dx / steps;
        double incrementY = dy / steps;

        // Break the move into manageable pieces.
        for (uint_fast16_t i = 0; (i < steps) && !AbortShape; i++)
        {
            // Calculate next intermediate point to move to.
            newX += incrementX;
            newY += incrementY;

            // Move the motors to the intermediate position.
            MoveTo(newX, newY);
        }

        // Update the current position.
        // Note that due to rounding of floats, it is better to re-calculate
        // our current position based on in/out and rotary values than to use
        // newX and newY here
        CalculateXY();
    }
} // End GotoXY().


/////////////////////////////////////////////////////////////////////////////////
// RotateGotoXY()
//
// Move to a specified target point rotated about the origin by a specified
// radian value.  The move is made in a coordinated manner (with interpolation).
//
// Arguments:
//   - targetX, targetY : The cartesian coordinates of the target position.
//   - angle            : The angle (in radians) to rotate the specified point
//                        about the origin.
/////////////////////////////////////////////////////////////////////////////////
void RotateGotoXY(double targetX, double targetY, double angle)
{
    double x = targetX;
    double y = targetY;

    // Perform rotation if requested.
    if (angle != 0.0)
    {
        // Transform (rotate) the target coordinates.
        {
            x = targetX * cos(angle) - targetY * sin(angle);
            y = targetY * cos(angle) + targetX * sin(angle);
        }
    }

    // Move to the (possibly rotated) target location.
    GotoXY(x, y);
} // End RotatedGotoXY().


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
// accordingly.  Returns an indication of whether or not motors should run.
//
// Returns:
//    Returns 'true' if motors should continue running.  Returns 'false' otherwise.
/////////////////////////////////////////////////////////////////////////////////
bool UpdateSpeeds()
{
    // Assume we're going to keep running (i.e. not pausing).
    bool keepRunning = true;

    // Only read the right SPEED pot if we're not being controlled remotely.
    if (!RemoteSpeedDelay)
    {
        // Pot on the right for drawing speed or delay.  Read it now.
        uint_fast16_t speedKnobVal = ReadAPot(SPEED_POT_PIN);
        SpeedDelay = map(speedKnobVal, KNOB_MIN_VAL, KNOB_MAX_VAL,
                         SPEED_DELAY_MAX_VAL,  SPEED_DELAY_MIN_VAL);
        // Check against a value slightly higher than 0 due to ADC inaccuracies.
        keepRunning = (speedKnobVal >= KNOB_MIN_VAL + 16);
    }

    // Set the values atomically.
    noInterrupts();
    RotDelay   = SpeedDelay * RotSpeedFactor;
    InOutDelay = SpeedDelay * InOutSpeedFactor;
    interrupts();

    // Indicate if we're pausing.
    digitalWrite(PAUSE_LED_PIN, !keepRunning || RemotePause);

    // Return an indication of whether or not motors should run.
    return keepRunning;
} // End UpdateSpeeds().


/////////////////////////////////////////////////////////////////////////////////
// ReadPots()
//
// Updates the brightness and speed values by reading the respective pots and
// setting the corresponding values based on the pot values.  If the speed pot
// is detected as being zero, a pause state is entered.  In this state, the axes
// are prohibited from moving.  This state remains in effect until the speed pot
// is no longer set to zero.
/////////////////////////////////////////////////////////////////////////////////
void ReadPots()
{
    // Perform any requested remote commands.
    HandleRemoteCommands();

    // Update the LED brightness based on pot input.
    UpdateLeds();

    // Update the speeds based on pot input and check for pause.
    // If pause, wait for the pause to go away.
    while (!UpdateSpeeds() || RemotePause)
    {
        // We are in the pause state, disable motor ISRs execution.
        Pausing = true;

        // Keep checking for remote commands.
        HandleRemoteCommands();

        // Continue to update the LED brightness.
        UpdateLeds();
    }

    // No longer paused, exit re-nable the motors ISRs.
    Pausing = false;
} // End ReadPots();


/////////////////////////////////////////////////////////////////////////////////
// Display()
//
// If LOG_VERBOSE is 'true', print some useful debugging information.
/////////////////////////////////////////////////////////////////////////////////
void Display()
{
    LOG_F(LOG_VERBOSE, "%d \\ %d,%d, * ", MRPointCount, RotDelay, InOutDelay);
    LOG_F(LOG_VERBOSE, "%d,%d * ", InOutOn, RotOn);
    LOG_F(LOG_VERBOSE, "%.1f x,y %.1f", CurrentX, CurrentY);
    LOG_F(LOG_VERBOSE, " %d>%d %d>%d\n", RotSteps, RotStepsTo, InOutSteps, InOutStepsTo);
} // End Display().


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
        while (UpdateSpeeds())
        {
            UpdateLeds();
            delay(1);
        }

        // Now wait for speed pot pause to be cleared.
        while (!UpdateSpeeds())
        {
            UpdateLeds();
            delay(1);
        }
    }

    // Delay a bit to allow time to set speed pot back to desired value.
    const uint_fast32_t END_PLOT_DELAY = 3000;   // 3 second delay.
    uint_fast32_t startTime = millis();
    while (millis() - startTime < END_PLOT_DELAY)
    {
        delayMicroseconds(50);
        ReadPots();
    }
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
// GenerateSeriesSteps()
//
// Generates a random number of steps, size increment, and rotation increment to
// be used in series functions.  This behavior is common to all series functions,
// so has been pulled out as a separate function.
//
// Arguments:
//   - size    : Initial size of the shape.
//   - steps   : Number of steps for the series.  This value is generated here
//               and returned to the caller.
//   - sizeInc : Size increment to use for each step.  This value is generated
//               here and returned to the caller.
//   - rotInc  : Rotation increment to use for each step.  This value is
//               generated here and returned to the caller.
//
// Returns:
//   The generated steps, sizeInc, and rotInc values are returned to the caller.
/////////////////////////////////////////////////////////////////////////////////
void GenerateSeriesSteps(uint_fast16_t size, uint_fast16_t &steps,
                         uint_fast16_t &sizeInc, double &rotInc)
{
    // Determine how many steps to take, and how much to increase size and angle.
    steps   = random(MIN_SERIES_STEPS, MAX_SERIES_STEPS);
    sizeInc = (MAX_SCALE_I - size) / steps;
    sizeInc = max(sizeInc, MIN_SERIES_INC);
    steps   = 1 + (MAX_SCALE_I - size) / sizeInc;
    rotInc  = RandomFloat(0.0, DtoR(MAX_SERIES_ANGLE));
    LOG_F(LOG_INFO, "(%d,%d,%d,%.2f)\n", size, steps, sizeInc, rotInc);
} // End GenerateSeriesSteps().


/////////////////////////////////////////////////////////////////////////////////
// S H A P E   F U N C T I O N S
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Circle()
//
// Draw a circle, ellipse, or lissajous curve.
//
// Arguments:
//   - numLobes : For circle or ellipse, set to 1.  For lissajous, specifies
//                the number of lobes to display.
//   - xSize    : Size along the X axis.
//   - ySize    : Size along the Y axis.
//   - rotation : Amount to rotate the curve, in radians.
//
// Note: For circle, set xSize and ySize equal.  For ellipse, set xSize and
//       ySize unequal.
/////////////////////////////////////////////////////////////////////////////////
void Circle(uint_fast16_t numLobes, uint_fast16_t xSize,
            uint_fast16_t ySize, double rotation)
{
    // Make sure all arguments are within valid limits.
    numLobes = constrain(numLobes, MIN_CIRCLE_LOBES, MAX_CIRCLE_LOBES);
    xSize    = constrain(xSize, MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE);
    ySize    = constrain(ySize, MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE);

    // Show our call.
    LOG_F(LOG_INFO, "Circle(%d,%d,%d,%.1f)\n", numLobes, xSize, ySize, RtoD(rotation));

    // Loop to create the curve.
    for (uint_fast16_t i = 0; (i <= SPIRO_NUM_POINTS)  && !AbortShape; i++)
    {
        double angle = SPIRO_ANGLE_BASE * (double)i;
        double x = (double)xSize * cos(angle);
        double y = (double)ySize * sin(numLobes * angle);
        RotateGotoXY(x, y, rotation);
    }
} // End Circle().


/////////////////////////////////////////////////////////////////////////////////
// RandomCircle()
//
// Calls Circle() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomCircle()
{
    // Generate some legal arguments for the call to Circle().
    uint_fast16_t lobes = random(MIN_CIRCLE_LOBES, MAX_CIRCLE_LOBES + 1);
    uint_fast16_t xSize = random(MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE + 1);
    uint_fast16_t ySize = random(MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE + 1);
    double       rot   = RandomFloat(0.0, PI);

    // Make the call to Circle().
    Circle(lobes, xSize, ySize, rot);
} // End RandomCircle().


/////////////////////////////////////////////////////////////////////////////////
// EllipseSeries()
//
// Create a series of ellipses vith varying size and rotation.
/////////////////////////////////////////////////////////////////////////////////
void EllipseSeries()
{
    LOG_F(LOG_INFO, "EllipseSeries");

    // Usefull constants.
    const uint_fast16_t MIN_LOBES = 1;   // Need at least one lobe.
    const uint_fast16_t MAX_LOBES = 2;   // For series, no more than 2 lobes.

    // Generate some legal arguments for the calls to Circle().
    uint_fast16_t lobes  = random(MIN_LOBES, MAX_LOBES + 1);
    uint_fast16_t xSize  = random(MIN_ELLIPSE_SIZE, MAX_SCALE_I);
    double       ratio  = RandomFloat(MIN_ELLIPSE_RATIO, MAX_ELLIPSE_RATIO);
    uint_fast16_t ySize  = (uint_fast16_t)(xSize / ratio);
                  ySize  = constrain(ySize, MIN_ELLIPSE_SIZE, MAX_SCALE_I);
    double       rot    = RadAngle;

    // Determine how many steps to take, and how much to increase size and
    uint_fast16_t steps = 0;
    uint_fast16_t sizeInc = 0;
    double       rotInc = 0.0;
    GenerateSeriesSteps(xSize, steps, sizeInc, rotInc);

    // Loop to create the ellipse series.
    for (uint_fast16_t i = 0; (i < steps) && !AbortShape; i++)
    {
        // Make the call to Circle() and increment our size and rotation.
        Circle(lobes, xSize, ySize, rot);
        xSize += sizeInc;
        ySize += sizeInc;
        rot += rotInc;
    }
    EndSeries();
} // End EllipseSeries().


/////////////////////////////////////////////////////////////////////////////////
// ClearFromIn()
//
// Clear (wipe) the board in a spiral from inside to outside.
//
// Arguments:
//   The anonymous argument is unused.
/////////////////////////////////////////////////////////////////////////////////
void ClearFromIn(double = 0.0)
{
    LOG_F(LOG_INFO, "ClearFromIn()\n");

    // Moe to our start point (0, 0).
    GotoXY(0, 0);

    // Take care of any position loss by forcing more IN.
    ForceInOut(1, IN, MAX_FORCE_DELAY);

    // Do the wipe.
    MotorRatios(WIPE_RATIO, false);
} // End ClearFromIn().


/////////////////////////////////////////////////////////////////////////////////
// ClearFromOut()
//
// Clear (wipe) the board in a spiral from outside to inside.
//
// Arguments:
//   The anonymous argument is unused.
/////////////////////////////////////////////////////////////////////////////////
void ClearFromOut(double = 0.0)
{
    LOG_F(LOG_INFO, "ClearFromOut()\n");

    // Move the in/out arm all the way out.
    ExtendInOut();

    MotorRatios(WIPE_RATIO, false);
} // End ClearFromOut().


/////////////////////////////////////////////////////////////////////////////////
// ClearLeftRight()
//
// Clear (wipe) the board via raster (back and forth) movements.
//
// Arguments:
//   - rotation : How much to rotate the raster, in radians.
/////////////////////////////////////////////////////////////////////////////////
void ClearLeftRight(double rotation = 0.0)
{
    LOG_F(LOG_INFO, "ClearLeftRight(%.1f)\n", RtoD(rotation));

    // Clear from left and right
    for (int_fast16_t i = -(int_fast16_t)MAX_SCALE_I;
         (i <= (int_fast16_t)MAX_SCALE_I) && !AbortShape; i += WIPE_RASTER_INC)
    {
        ReadPots();

        // Calculate xt based on circle equation.
        double xt = sqrt((MAX_SCALE_F * MAX_SCALE_F) - (i * i));
        RotateGotoXY(xt,  i, rotation);
        RotateGotoXY(-xt, i, rotation);
        double newY = i + WIPE_RASTER_INC / 2;
        if (newY <= MAX_SCALE_F)
        {
            // Move to -xt, newY with rotation.
            RotateGotoXY(-xt, newY, rotation);
            // Move back to xt, newY with rotation.
            RotateGotoXY(xt, newY, rotation);
        }
    }
} // End ClearLeftRight().


/////////////////////////////////////////////////////////////////////////////////
// Wipes[]
//
// An array of pointers to functions that perform wipes on the table (i.e. they
// clear/erase the table).
/////////////////////////////////////////////////////////////////////////////////
void (*const Wipes[])(double) =
{
    ClearFromIn,
    ClearFromOut,
    ClearLeftRight
}; // End Wipes[].


/////////////////////////////////////////////////////////////////////////////////
// RandomWipe()
//
// Executes a randomly selected wipe.
//
// Note: This function also homes the axes before performing the wipe in order
//       to keep the axes in sync.
/////////////////////////////////////////////////////////////////////////////////
void RandomWipe()
{
    // Start at the origin to eliminate noise when in/out fully extends.
    GotoXY(0, 0);

    // Perform a home to keep in axes in sync since we can occasionally
    // lose position.
    Home();

    // Select a random wipe and execute it.
    size_t index = random(0, sizeof(Wipes) / sizeof(Wipes[0]));
    (*Wipes[index])(RandomFloat(0.0, PI));
} // End RandomWipe().


/////////////////////////////////////////////////////////////////////////////////
// Clover()
//
// Draw a clover shape (epicycloid).  Imagine two circles, with an outer
// circle rolling around an inner one.  The path created by a point on
// the outer circle as it rolls is called an epicycloid.
//
// Arguments:
//   - fixedR : Relative size of the fixed outer circle (1 - 10).
//   - outerR : Relative size of the outer circle. (1 - 10)
//   - xSize  : Size along the X axis.
//   - ySize  : Size along the Y axis.
//   - res    : Resolution of points per loop.  Larger values generate smoother
//              curves.
/////////////////////////////////////////////////////////////////////////////////
void Clover(uint_fast16_t fixedR, uint_fast16_t outerR, uint_fast16_t xSize,
            uint_fast16_t ySize,  uint_fast16_t res)
{
    // Limit our input values.
    fixedR = constrain(fixedR, MIN_CLOVER_VAL, MAX_CLOVER_VAL);
    outerR = constrain(outerR, MIN_CLOVER_VAL, MAX_CLOVER_VAL);
    res    = constrain(res,    MIN_CLOVER_RES, MAX_CLOVER_RES);

    // Show our call.  This may come in handy.  If an interesting path is displayed,
    // the arguments may be used again.
    LOG_F(LOG_INFO, "Clover(%d,%d,%d,%d,%d)\n", fixedR, outerR, xSize, ySize, res);

    // Reduce the radii values.
    Reduce(fixedR, outerR);

    // Setup some variables.
    double baseAngle = PI_X_2 / res;
    double rSum      = (double)(fixedR + outerR);
    xSize = constrain(xSize, MIN_CLOVER_SIZE, MAX_SCALE_I) / (fixedR + 2 * outerR);
    ySize = constrain(ySize, MIN_CLOVER_SIZE, MAX_SCALE_I) / (fixedR + 2 * outerR);

    // Calculate the number of cycles to complete the plot.
    double    angleFactor = rSum / outerR;
    uint_fast32_t cycles = 1;
    if (fixedR % outerR != 0)
    {
        cycles = LCM(fixedR, outerR) / fixedR;
        cycles = min(cycles, MAX_CYCLES);
    }

    // Rotate our shape so that the start point is as close as possible to the
    // current ball position.  Since we always start our shape with a 0 degree angle,
    // we can use the current position angle as the amount we need to rotate.
    double offsetAngle = RadAngle;

    // Loop to generate the plot.
    for (uint_fast32_t i = 0; (i <= res * cycles) && !AbortShape; i++)
    {
        // Print the cycle countdown if verbose mode.  Note that this statement
        // will be completely optimized out if LOG_CYCLES == 0.
        if (LOG_CYCLES && (i % res == 0))
        {
            LOG_F(LOG_CYCLES, "%d\n", cycles - i / res);
        }

        // Calculate the angle for this point.
        double angle = baseAngle * (double)i;

        // Calculate the Spirograph coordinates.
        double x = rSum * cos(angle) - outerR * cos(angleFactor * angle);
        double y = rSum * sin(angle) - outerR * sin(angleFactor * angle);

        // Move the drawing arm to the calculated coordinates.
        RotateGotoXY(xSize * x, ySize * y, offsetAngle);
    }
} // End Clover().


/////////////////////////////////////////////////////////////////////////////////
// RandomClover()
//
// Calls Clover() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomClover()
{
    // Generate some legal arguments for the call to Clover().
    uint_fast16_t a = random(MIN_CLOVER_VAL, MAX_CLOVER_VAL + 1);
    uint_fast16_t b = random(MIN_CLOVER_VAL, MAX_CLOVER_VAL + 1);

    // Make sure our random values are not equal.
    while (a == b)
    {
        b = random(MIN_CLOVER_VAL, MAX_CLOVER_VAL + 1);
    }

    uint_fast16_t xSize = random(MIN_CLOVER_SIZE, MAX_SCALE_I + 1);
    uint_fast16_t ySize = random(MIN_CLOVER_SIZE, MAX_SCALE_I + 1);
    uint_fast16_t res   = random(MIN_CLOVER_RES,  MAX_CLOVER_RES) + 1;

    // Make the call to Rose().
    Clover(a, b, xSize, ySize, res);
} // End RandomClover().


/////////////////////////////////////////////////////////////////////////////////
// Heart()
//
// Draws a heart of specified size and rotation.
//
// Arguments:
//   - size     : Size of the heart.
//   - rotation : How much to rotate the heart, in radians.
//   - res      : Resolution of points per loop.  Larger values generate smoother
//                curves.
/////////////////////////////////////////////////////////////////////////////////
void Heart(uint_fast16_t size, double rotation, uint_fast16_t res)
{
    // Make sure the heart fits within the table.
    double scale     = 0.9 * (double)constrain(size, MIN_HEART_SIZE, MAX_SCALE_I);
    res               = constrain(res, MIN_HEART_RES, MAX_HEART_RES);
    double baseAngle = PI_X_2 / (double)res;

    LOG_F(LOG_INFO, "Heart(%d,%.1f,%d)\n", size, RtoD(rotation), res);

    // Loop to create the heart.
    for (uint_fast16_t i = 0; (i <= res) && !AbortShape; i++)
    {
        double angle = baseAngle * (double)i;
        double x = scale * pow(sin(angle), 3.0);
        double y = scale *
                    ((13.0 / 16.0) * cos(angle) +
                    (-5.0 / 16.0) * cos(2.0 * angle) +
                    (-2.0 / 16.0) * cos(3.0 * angle) +
                    (-1.0 / 16.0) * cos(4.0 * angle));
        RotateGotoXY(x, y, rotation);
    }
} // End Heart().


/////////////////////////////////////////////////////////////////////////////////
// HeartSeries()
//
// Create a series of hearts vith varying size and rotation.
/////////////////////////////////////////////////////////////////////////////////
void HeartSeries()
{
    LOG_F(LOG_INFO, "HeartSeries");

    // Generate some legal arguments for the calls to Heart().
    uint_fast16_t size = random(MIN_HEART_SIZE, (3 * MAX_SCALE_I / 4) + 1);
    double       rot  = RadAngle;
    uint_fast16_t res  = random(MIN_HEART_RES, MAX_HEART_RES + 1);

    // Determine how many steps to take, and how much to increase size and angle.
    uint_fast16_t steps   = 0;
    uint_fast16_t sizeInc = 0;
    double       rotInc  = 0.0;
    GenerateSeriesSteps(size, steps, sizeInc, rotInc);

    // Loop to create the Heart series.
    for (uint_fast16_t i = 0; (i < steps) && !AbortShape; i++)
    {
        // Make the call to Heart() and increment our size and rotation.
        Heart(size, rot, res);
        size += sizeInc;
        rot += rotInc;
    }
    EndSeries();
} // End HeartSeries().


/////////////////////////////////////////////////////////////////////////////////
// MotorRatios()
//
// Continuously move the motors at the specified ratios.
//
// Arguments:
//   - ratio    : Relative ratio of speed of the in/out axis to the speed of the
//                rotary axis.  Valid values range from 1 / MAX_MOTOR_RATIO
//                to MAX_MOTOR_RATIO.
//   - multiplePoints : Set 'true' if multiple points are desired.  Set 'false'
//                otherwise.
//   - inLimit  : Sets the inner limit for direction changes and finishing.
//                Uses user scale units (0 through MAX_SCALE * .8).
//   - outLimit : Sets the outer limit for direction changes and finishing.
//                Uses user scale units (inLimit + 10 through MAX_SCALE).
//
// Note: - The motors will move at a rate of 10 / ratio.  The factor of 10 is due
//         to the 10:1 gear ratio used for the rotaty axis.
//       - If 'multiplePoints' is 'true', then the in/out axis will reverse direction
//         when reaching the positive or zero extent of the arm.  It will
//         continue doing this until a calculated number of points has been
//         created.  If 'multiplePoints' is 'false', the curve will terminate once
//         the in/out arm reaches either of its extents.
/////////////////////////////////////////////////////////////////////////////////
void MotorRatios(double ratio, bool multiplePoints, int_fast16_t inLimit,
                 int_fast16_t outLimit)
{
    // Make sure we have a valid ratio.
    ratio = constrain(ratio, MIN_MOTOR_RATIO, MAX_MOTOR_RATIO);

    // Update our variable in/out limits.
    inLimit  = constrain(inLimit,  0, (int_fast16_t)(8 * MAX_SCALE_I / 10));
    outLimit = constrain(outLimit, inLimit + 10, (int_fast16_t)MAX_SCALE_I);

    // Show our call.
    LOG_F(LOG_INFO, "MotorRatios(%.1f,%d,%d,%d)\n", ratio, multiplePoints, inLimit, outLimit);

    // Convert our in out units to in/out motor steps.
    InLimit  = ((int_fast32_t)inLimit  * (int_fast32_t)INOUT_TOTAL_STEPS) /
                (int_fast32_t)MAX_SCALE_I;
    OutLimit = ((int_fast32_t)outLimit * (int_fast32_t)INOUT_TOTAL_STEPS) /
                (int_fast32_t)MAX_SCALE_I;

    // Initialize the speed factors for the motors based on our ratio argument.
    if (ratio >= 1.0)
    {
        SetSpeedFactors(1.0, ratio);
    }
    else
    {
        SetSpeedFactors(1.0 / ratio, 1.0);
    }

    // Try to guess how many multiple points to generate.
    // This calculation could use some work.
    uint_fast32_t cycles = 1;
    if (multiplePoints)
    {
        if (ratio > 1.0)
        {
            if (ratio <= 5.0)
            {
                cycles = 24;
            }
        }
        else
        {
            cycles = min((uint_fast32_t)(40.0 / ratio), 2 * MAX_CYCLES);
        }
    }

    // Get ready to do the move by setting the points count.
    MRPointCount = cycles;

    // Select the direction based on our current position.  If we're closer to
    // the positive limit, then go in.  Otherwise go out.
    if (InOutSteps > INOUT_TOTAL_STEPS / 2)
    {
        DirInOut = IN;
        DirRot = !ROT_CAUSING_IN;
    }
    else
    {
        DirInOut = OUT;
        DirRot = ROT_CAUSING_IN;
    }

    // Start the move by turning the motors on atomically.
    noInterrupts();
    RotOn   = true;
    InOutOn = true;
    interrupts();

    // Wait for the move to complete.  ISR will turn off RotOn and InOutOn when
    // MRPointCount equals 0;
    while ((RotOn || InOutOn) && !AbortShape)
    {
        ReadPots();
        CalculateXY();       // Keep track of the location as it moves.
        Display();

        // If LOG_CYCLES is 'false' then this entire block wiil be optimized out.
        if (LOG_CYCLES)
        {
            static uint_fast16_t lastMRPoints = 0;
            if (lastMRPoints != MRPointCount)
            {
                LOG_F(LOG_CYCLES,"%d\n", MRPointCount);
                lastMRPoints = MRPointCount;
            }
        }
    }

    // Restore the default in/out limits and speed factors.
    InLimit  = 0;
    OutLimit = MAX_SCALE_I;
    SetSpeedFactors(1.0, 1.0);
    CalculateXY();       // Update our current position variables.
} // End MotorRatios().


/////////////////////////////////////////////////////////////////////////////////
// RandomRatios()
//
// Calls MotorRatios() with a random ratio.
/////////////////////////////////////////////////////////////////////////////////
void RandomRatios()
{
    // Generate a normalized ratio value for the call to MotorRatios().
    double ratio = RandomFloat(1.0, MAX_MOTOR_RATIO / 3.0);

    // Randomly select whether to generate a value less than 1.0.  Could have
    // selected a random ratio from (1.0 / MAX_MOTOR_RATIO) to MAX_MOTOR_RATIO / 3.0.
    // But this heavily biases the result to a value greater than 1.0.  Instead
    // randomly re generate a value 0.1 and 0.9.  This is much more fair.
    if (RandomBool())
    {
        ratio = RandomFloat(0.1 / (MAX_MOTOR_RATIO / 3.0), 1.0);
    }

    // Make the call to MotorRatios().
    MotorRatios(ratio, true);
} // End RandomRatios().


/////////////////////////////////////////////////////////////////////////////////
// RandomRatiosRing()
//
// Calls MotorRatios() with a random ratio and random width.  Make a ring.
/////////////////////////////////////////////////////////////////////////////////
void RandomRatiosRing()
{
    // Generate a normalized ratio value for the call to MotorRatios().
    double ratio = RandomFloat(1.0, MAX_MOTOR_RATIO / 3.0);

    // Randomly select whether to generate a value less than 1.0.  Could have
    // selected a random ratio from (1.0 / MAX_MOTOR_RATIO) to MAX_MOTOR_RATIO / 3.0.
    // But this heavily biases the result to a value greater than 1.0.  Instead
    // randomly re generate a value 0.1 and 0.9.  This is much more fair.
    if (RandomBool())
    {
        ratio = RandomFloat(0.1 / (MAX_MOTOR_RATIO / 3.0), 1.0);
    }

    int_fast16_t inLimit  = random(0, (int_fast16_t)(0.8 * MAX_SCALE_F) + 1);
    int_fast16_t outLimit = random(inLimit + 10, (int_fast16_t)MAX_SCALE_I + 1);

    // Show our call.
    LOG_F(LOG_INFO, "MotorRatios(%f,true,%d,%d)\n", ratio, inLimit, outLimit);

    // Make the call to MotorRatios().
    MotorRatios(ratio, true, inLimit, outLimit);
} // End RandomRatiosRing().


/////////////////////////////////////////////////////////////////////////////////
// Plots[]
//
// This array of PlotInfo instances holds data pertaining to all the known plot
// arrays.
/////////////////////////////////////////////////////////////////////////////////
const PlotInfo Plots[] =
{
    {JMCPlot,  sizeof(JMCPlot)  / sizeof(JMCPlot[0]),  false, "JMCPlot"},
    {MazePlot, sizeof(MazePlot) / sizeof(MazePlot[0]), true,  "MazePlot"}
}; // End Plots[].


/////////////////////////////////////////////////////////////////////////////////
// PlotShapeArray()
//
// Plots a shape based on an array of Coordinate values.
//
// Arguments:
//   - shape  : A pointer to the array of Coordinate structs for the shape.
//   - size   : The size of the array (= # Coordinate instances).
//   - rotate : Set 'true' to allow rotation.  'false' otherwise.
//   - pName  : Name string of the array to plot.
/////////////////////////////////////////////////////////////////////////////////
void PlotShapeArray(const Coordinate shape[], uint_fast16_t size, bool rotate,
                    const char *pName)
{
    // Show our call.
    LOG_F(LOG_INFO, "PlotShapeArray(%s,%d,%d)\n", pName, size, rotate);

    // Rotate our shape so that the start point is as close as possible to the
    // current ball position.
    double rotation = (rotate ? (RadAngle - atan2(shape[0].y, shape[0].x)) : 0.0);

    // Simply loop through the array, plotting each Coordinate.
    for (size_t i = 0; (i < size) && !AbortShape; i++)
    {
        // Update LEDs and speeds.
        ReadPots();

        // Pick out the X and Y values and go to them.
        double x = shape[i].x;
        double y = shape[i].y;
        RotateGotoXY(x, y, rotation);

        // NOTE: Here is a rudimentary single stepping mechanism.  It can be used
        //       determine redundant/useless coordinates that may be removed from
        //       a new shape array.  These can then be removed in order to
        //       conserve memory.  It will be completely optimized out if
        //       LOG_STEP == 0.
        if (LOG_STEP)
        {
            // Display the current index and position.
            LOG_F(LOG_STEP, "%d   %.1f,%.1f\n", i, x, y);

            // Loop to handle step commands.  Space to step to the next coordinate,
            // and '-' to step to the previous coordinate.
            while(1)
            {
                // Be sure to update our LEDs and speeds.
                ReadPots();

                // Handle step commands.  Ignore all other characters.  Note that
                // the only way to exit the while loop is via one of the step
                // characters.
                if (Serial.available() > 0)
                {
                    char c = Serial.read();
                    if (c == ' ')
                    {
                        break;
                    }
                    else if ((c == '-') && (i > 0))
                    {
                        i -= 2;
                        break;
                    }
                } // End if
            } // End while
        } // End LOG_STEP
    } // End for().
} // End PlotShapeArray().


/////////////////////////////////////////////////////////////////////////////////
// RandomPlot()
//
// Displays a random plot array.
/////////////////////////////////////////////////////////////////////////////////
void RandomPlot()
{
    // Select a random one and execute it.
    size_t index = random(0, sizeof(Plots) / sizeof(Plots[0]));
    PlotShapeArray(Plots[index].m_Plot, Plots[index].m_Size, Plots[index].m_Rotate,
                   Plots[index].m_pName);
} // End RandomPlot().


/////////////////////////////////////////////////////////////////////////////////
// Polygon()
//
// Create a polygon of a specific size.
//
// Arguments:
//   - numSides : Number of sides the polygon will have.
//   - size     : Size of the polygon.
//   - rotation : How much to rotate the polygon, in radians.
/////////////////////////////////////////////////////////////////////////////////
void Polygon(uint_fast16_t numSides, uint_fast16_t size, double rotation)
{
    // Make sure all arguments are within valid limits.
    numSides      = constrain(numSides, MIN_POLY_SIDES, MAX_POLY_SIDES);
    double scale = (double)constrain(size, MIN_POLY_SIZE, MAX_SCALE_I);

    LOG_F(LOG_INFO, "Polygon(%d,%d,%.1f)\n", numSides, size, RtoD(rotation));

    // Loop to create the (possibly rotated) polygon.
    for (uint_fast16_t i = 0; (i <= numSides) && !AbortShape; i++)
    {
        double angle = rotation + (PI_X_2 * (double)i) / (double)numSides;
        GotoXY(scale * cos(angle), scale * sin(angle));
    }
} // End Polygon().


/////////////////////////////////////////////////////////////////////////////////
// PolygonSeries()
//
// Create a series of polygons vith varying size and rotation.
/////////////////////////////////////////////////////////////////////////////////
void PolygonSeries()
{
    LOG_F(LOG_INFO, "PolygonSeries");

    // Generate some legal arguments for the calls to Polygon().
    uint_fast16_t sides = random(MIN_POLY_SIDES, MAX_POLY_SIDES + 1);
    uint_fast16_t size  = random(MIN_POLY_SIZE, (3 * MAX_SCALE_I / 4) + 1);
    double       rot   = RadAngle;

    // Determine how many steps to take, and how much to increase size and angle.
    uint_fast16_t steps = 0;
    uint_fast16_t sizeInc = 0;
    double       rotInc = 0.0;
    GenerateSeriesSteps(size, steps, sizeInc, rotInc);

    // Loop to create the polygon series.
    for (uint_fast16_t i = 0; (i < steps) && !AbortShape; i++)
    {
        // Make the call to Polygon() and increment our size and rotation.
        Polygon(sides, size, rot);
        size += sizeInc;
        rot += rotInc;
    }
    EndSeries();
} // End PolygonSeries().


/////////////////////////////////////////////////////////////////////////////////
// RandomLines()
//
// Create a random number of random lines.
/////////////////////////////////////////////////////////////////////////////////
void RandomLines()
{
    LOG_F(LOG_INFO, "RandomLines()\n");

    // Determine how many lines to generate.
    uint_fast16_t numPoints = random(MIN_RANDOM_POINTS, MAX_RANDOM_POINTS);

    // Generate the random lines.
    for (uint_fast16_t i = numPoints; i; i--)
    {
        LOG_F(LOG_INFO, "%d\n", i);

        GotoXY(random(-(int_fast16_t)MAX_SCALE_I, (int_fast16_t)MAX_SCALE_I + 1),
               random(-(int_fast16_t)MAX_SCALE_I, (int_fast16_t)MAX_SCALE_I + 1));
    }
} // End RandomLines().


/////////////////////////////////////////////////////////////////////////////////
// Rose()
//
// Draw a rose curve is a curve which has the shape of a petalled flower.
//
// Arguments:
//   - num   : Numerator.
//   - denom : Denominator.
//   - xSize : Size along the X axis.
//   - ySize : Size along the Y axis.
//   - res   : Resolution of points per loop.  Larger values generate smoother
//             curves.
/////////////////////////////////////////////////////////////////////////////////
void Rose(uint_fast16_t num, uint_fast16_t denom, uint_fast16_t xSize,
          uint_fast16_t ySize, uint_fast16_t res)
{
    // Limit input values to legal range.
    num   = constrain(num,   MIN_ROSE_VAL, MAX_ROSE_VAL);
    denom = constrain(denom, MIN_ROSE_VAL, MAX_ROSE_VAL);
    xSize = constrain(xSize, MIN_ROSE_SIZE, MAX_SCALE_I);
    ySize = constrain(ySize, MIN_ROSE_SIZE, MAX_SCALE_I);
    res   = constrain(res,   MIN_ROSE_RES, MAX_ROSE_RES);

    // Show our call.
    LOG_F(LOG_INFO, "Rose(%d,%d,%d,%d,%d)\n", num, denom, xSize, ySize, res);

    Reduce(num, denom);

    // If exactly one of the numerator and denominator is odd, then we need to
    // double the number of cycles.
    uint_fast16_t parity = ((num & 1) ^ (denom & 1)) + 1;

    uint_fast16_t cycles = num * parity * res;
    double baseAngle1 = (double)num / (double)denom;
    double baseAngle2 = PI * denom / res / num;

    // Loop to create the curve.
    for (uint_fast16_t i = 0; (i <= cycles) && !AbortShape; i++)
    {
        if (LOG_CYCLES && (i % res == 0))
        {
            LOG_F(LOG_CYCLES, "%d\n", (cycles - i) / res);
        }

        double theta = baseAngle2 * i;
        double r = sin(baseAngle1 * theta);
        double x = xSize * r * cos(theta);
        double y = ySize * r * sin(theta);
        GotoXY(x, y);
    }
} // End Rose().


/////////////////////////////////////////////////////////////////////////////////
// RandomRose()
//
// Calls Rose() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomRose()
{
    // Generate some legal arguments for the call to Rose().
    uint_fast16_t n = random(MIN_ROSE_VAL, MAX_ROSE_VAL + 1);
    uint_fast16_t d = random(MIN_ROSE_VAL, MAX_ROSE_VAL + 1);

    // Make sure our random values are not equal.
    while (d == n)
    {
        d = random(MIN_ROSE_VAL, MAX_ROSE_VAL);
    }

    uint_fast16_t xSize = random(MIN_ROSE_SIZE, MAX_SCALE_I + 1);
    uint_fast16_t ySize = random(MIN_ROSE_SIZE, MAX_SCALE_I + 1);
    uint_fast16_t res   = random(MIN_ROSE_RES,MAX_ROSE_RES + 1 );

    // Make the call to Rose().
    Rose(n, d, xSize, ySize, res);
} // End RandomRose().


/////////////////////////////////////////////////////////////////////////////////
// Spirograph()
//
// Simulate a spirograph with a fixed outer wheel, a rolling inner wheel, and
// an offset from the rolling inner wheel.  The result is a cycloid shape that is
// highly dependent on the 3 argument values.
//
// Arguments:
//    - fixedR : Radius of fixed outer circle.
//    - r      : Radius of the rolling circle.
//    - a      : Distance from the center of the rolling circle.
/////////////////////////////////////////////////////////////////////////////////
void Spirograph (uint_fast16_t fixedR, uint_fast16_t r, uint_fast16_t a)
{
    // Limit arguments to usable values.
    fixedR = constrain(fixedR, MIN_SPIRO_FIXEDR, MAX_SCALE_I);
    r      = constrain(r, MIN_SPIRO_SMALLR, fixedR - 5);
    a      = constrain(a, MIN_SPIRO_SMALLR, MAX_SCALE_I / 2);

    // Show our call.
    LOG_F(LOG_INFO, "Spirograph(%d,%d,%d)\n", fixedR, r, a);

    double rDiff      = (double)(fixedR - r); // Difference between fixed radius and r.
    double rDiffRatio = rDiff / r;             // Ratio for later use.

    // Calculate the number of cycles to complete the plot.
    uint_fast16_t larger = max(fixedR, r);
    uint_fast32_t cycles = LCM(fixedR, r) / larger;
    cycles = min(cycles, MAX_CYCLES);

    // Rotate our shape so that the start point is as close as possible to the
    // current ball position.  Since we always start our shape with a 0 degree angle,
    // we can use the current position angle as the amount we need to rotate.
    double offsetAngle = RadAngle;

    // Loop to generate the plot.
    for (uint_fast16_t i = 0; (i <= SPIRO_NUM_POINTS * cycles) && !AbortShape; i++)
    {
        // Print the cycle countdown if verbose mode.  Note that this statement
        // will be completely optimized out if LOG_CYCLES == 0.
        if (LOG_CYCLES && (i % SPIRO_NUM_POINTS == 0))
        {
            LOG_F(LOG_CYCLES, "%d\n", cycles - i / SPIRO_NUM_POINTS);
        }

        // Calculate the angle for this point.
        double angle = SPIRO_ANGLE_BASE * (double)i;

        // Calculate the Spirograph coordinates.
        double x = rDiff * cos(angle) + a * cos(rDiffRatio * angle);
        double y = rDiff * sin(angle) - a * sin(rDiffRatio * angle);

        // Move the drawing arm to the calculated coordinates.
        RotateGotoXY(x, y, offsetAngle);
    }
} // End Spirograph().


/////////////////////////////////////////////////////////////////////////////////
// RandomSpirograph()
//
// Calls Spirograph() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomSpirograph()
{
    // Generate some legal arguments for the call to Spirograph().
    uint_fast16_t fixedR = random(MIN_SPIRO_FIXEDR, MAX_SCALE_I + 1);
    uint_fast16_t r      = random(MIN_SPIRO_SMALLR, fixedR - 5);
    uint_fast16_t a      = random(MIN_SPIRO_SMALLR, MAX_SCALE_I / 2);

    // Make the call to Spirograph().
    Spirograph(fixedR, r, a);
} // End RandomSpirograph().


/////////////////////////////////////////////////////////////////////////////////
// Spirograph2()
//
// Simulate a spirograph with a fixed outer wheel, a rolling inner wheel, a
// second inner wheel, and an offset from the second rolling inner wheel.
// The result is a cycloid shape that is highly dependent on the 4 argument
// values.
//
// Arguments:
//    - fixedR : Radius of the fixed circle.
//    - r1     : Radius of the first rolling circle,.
//    - r2     : Radius of the second rolling circle.
//    - d      : Distance from the center of the second rolling circle.
/////////////////////////////////////////////////////////////////////////////////
void Spirograph2(uint_fast16_t fixedR, uint_fast16_t r1, uint_fast16_t r2,
                 uint_fast16_t d)
{
    // Limit arguments to usable values.
    fixedR = constrain(fixedR, MIN_SPIRO_FIXEDR, (8 * MAX_SCALE_I / 10));
    r1     = constrain(r1, MIN_SPIRO_SMALLR, fixedR - 3);
    r2     = constrain(r2, MIN_SPIRO_SMALLR, max(MIN_SPIRO_SMALLR + 1, r1 - 8));
    d      = constrain(d,  1, MAX_SCALE_I);

    // Show our call.
    LOG_F(LOG_INFO, "Spirograph2(%d,%d,%d,%d)\n", fixedR, r1, r2, d);

    double rDiff      = fixedR - r1;  // Difference between fixed radius and r1.
    double r12Diff    = r1 - r2;      // Difference between 2 radii of rolling circles.
    double baseAngle2 = rDiff / r1;   // Pre calculate to save loop execution time.
    double baseAngle3 = r12Diff / r2; // Pre calculate to save loop execution time.

    // Calculate the number of cycles to complete the plot.
    uint_fast16_t largest = max(max(fixedR, r1), d);
    uint_fast32_t cycles  = LCM(LCM(fixedR, r1), r2) / largest;
    cycles = min(cycles, MAX_CYCLES);

    // Rotate our shape so that the start point is as close as possible to the
    // current ball position.  Since we always start our shape with a 0 degree angle,
    // we can use the current position angle as the amount we need to rotate.
    double offsetAngle = RadAngle;

    // Loop to create the points of the cycloid.
    for (uint_fast16_t i = 0; (i <= SPIRO_NUM_POINTS * cycles) && !AbortShape; i++)
    {
        // Print the cycle countdown if verbose mode.  Note that this statement
        // will be completely optimized out if LOG_CYCLES == 0.
        if (LOG_CYCLES && (i % SPIRO_NUM_POINTS == 0))
        {
            LOG_F(LOG_CYCLES, "%d\n", cycles - i / SPIRO_NUM_POINTS);
        }

        // Calculate the angle for this point.
        double angle1 = SPIRO_ANGLE_BASE * (double)i;

        // Calculate the position of the first rolling circle (r1) around the fixed circle.
        double x1 = rDiff * cos(angle1);
        double y1 = rDiff * sin(angle1);

        // Calculate the angle for the second rolling circle (r2) relative to the
        // first rolling circle.
        double angle2 = baseAngle2 * angle1;

        // Calculate the position of the second rolling circle (r2) relative to
        // the first rolling circle and offset by the distance value (d).
        double angle3 = baseAngle3 * angle2;
        double x2 = x1 + r12Diff * cos(angle2) + d * cos(angle3);
        double y2 = y1 - r12Diff * sin(angle2) - d * sin(angle3);

        // Move the drawing arm to the calculated coordinates, rotated per
        // offsetAngle value.
        RotateGotoXY(x2, y2, offsetAngle);
    }
} // End Spirograph2().


/////////////////////////////////////////////////////////////////////////////////
// RandomSpirograph2()
//
// Calls Spirograph2() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomSpirograph2()
{
    // Generate some legal arguments for the call to Spirograph2().
    uint_fast16_t fixedR = random(MIN_SPIRO_FIXEDR, (8 * MAX_SCALE_I / 10) + 1);
    uint_fast16_t r1     = random(MIN_SPIRO_SMALLR, fixedR - 2);
    uint_fast16_t r2     = random(MIN_SPIRO_SMALLR, max(MIN_SPIRO_SMALLR + 1, r1 - 7));
    uint_fast16_t d      = random(1, MAX_SCALE_I + 1);

    // Make the call to Spirograph2().
    Spirograph2(fixedR, r1, r2, d);
} // End RandomSpirograph2().


/////////////////////////////////////////////////////////////////////////////////
// SpirographWithSquare()
//
// Simulate a spirograph with a fixed outer wheel, a rolling inner square, and
// an offset from the rolling inner square.  The result is a cycloid shape that is
// highly dependent on the 3 argument values.
//
// Arguments:
//    - fixedR : Radius of the fixed circle.
//    - s      : Side length of the square.
//    - d      : Offset from the center to the drawing tip.
/////////////////////////////////////////////////////////////////////////////////
void SpirographWithSquare(uint_fast16_t fixedR, uint_fast16_t s, uint_fast16_t d)
{
    // Limit arguments to usable values.
    fixedR = constrain(fixedR, MAX_SCALE_I / 5, (8 * MAX_SCALE_I / 10));
    s      = constrain(s, 1, fixedR / 2);
    d      = constrain(d, 1, MAX_SCALE_I);

    // Show our call.
    LOG_F(LOG_INFO, "SpirographWithSquare(%d,%d,%d)\n", fixedR, s, d);

    // Calculate the radius of the path traced by the center of the square.
    // Center of the square will move along a circle of radius (FIXED_R - s / 2).
    double halfS = (double)s / 2.0;
    double r = ((double)fixedR - halfS);
    // Pre-calculate a few values to minimize loop time.
    double rotAngleBase = (double)fixedR / (double)s;

    // Calculate the number of cycles to complete the plot.
    uint_fast16_t larger = max(fixedR, s);
    uint_fast32_t cycles = LCM(fixedR, s) / larger;
    cycles = min(cycles, MAX_CYCLES);

    // Start with an offset angle equal to the current position angle.
    double offsetAngle = RadAngle;

    // Loop to create the points of the cycloid.
    for (uint_fast16_t i = 0; (i <= SPIRO_NUM_POINTS * cycles) && !AbortShape; i++)
    {
        // Print the cycle countdown if verbose mode.  Note that this statement
        // will be completely optimized out if LOG_CYCLES == 0.
        if (LOG_CYCLES && (i % SPIRO_NUM_POINTS == 0))
        {
            LOG_F(LOG_CYCLES, "%d\n", cycles - i / SPIRO_NUM_POINTS);
        }

        // Calculate the angle for this point.
        double angle = SPIRO_ANGLE_BASE * (double)i;

        // Position of the square's center.
        double cx = r * cos(angle);
        double cy = r * sin(angle);

        // Calculate the angle of rotation of the square.
        double rotationAngle = rotAngleBase * angle;

        // Calculate the position of the drawing tip relative to the square.
        double tx = d * cos(rotationAngle) - halfS * sin(rotationAngle);
        double ty = d * sin(rotationAngle) + halfS * cos(rotationAngle);

        // Offset from the center of the square.
        double x = cx + tx;
        double y = cy + ty;

        // The first point is used to determine how much we must rotate the shape
        // in order to start at the closest point to the current position.
        if (i == 0)
        {
            offsetAngle = RadAngle - atan2(y, x);
        }

        // Move the drawing arm to the calculated coordinates, rotated by
        // offsetAngle value.
        RotateGotoXY(x, y, offsetAngle);
    }
} // End SpirographWithSquare().


/////////////////////////////////////////////////////////////////////////////////
// RandomSpirographWithSquare()
//
// Calls SpirographWithSquare() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomSpirographWithSquare()
{
    // Generate some legal arguments for the call to SpirographWithSquare().
    uint_fast16_t fixedR = random(MAX_SCALE_I / 5, (8 * MAX_SCALE_I / 10) + 1);
    uint_fast16_t s      = random(1, fixedR / 2 + 1);
    uint_fast16_t d      = random(1, MAX_SCALE_I + 1);

    // Make the call to SpirographWithSquare().
    SpirographWithSquare(fixedR, s, d);
} // End RandomSpirographWithSquare().


/////////////////////////////////////////////////////////////////////////////////
// Star()
//
// Create a star of specific size and shape.
//
// Arguments:
//   - numPoints : Number of star points.
//   - ratio     : Ratio of outer star points to inner star points.
//   - size      : Size of the star.
//   - rotation  : How much to rotate the star, in radians.
/////////////////////////////////////////////////////////////////////////////////
void Star(uint_fast16_t numPoints, double ratio, uint_fast16_t size,
          double rotation)
{
    // Make sure all arguments are within valid limits.
    numPoints = constrain(numPoints, MIN_STAR_POINTS, MAX_STAR_POINTS);
    ratio = constrain(ratio, MIN_STAR_RATIO, MAX_STAR_RATIO);
    size = (double)constrain(size, MIN_POLY_SIZE, MAX_SCALE_I);

    LOG_F(LOG_INFO, "Star(%d,%.1f,%d,%.1f)\n", numPoints, ratio, size, RtoD(rotation));

    // Loop to create the (possibly rotated) star.
    for (uint_fast16_t i = 0; (i <= numPoints * 2) && !AbortShape; i++)
    {
        double angle = rotation + (PI * (double)i) / (double)numPoints;
        double scale = size * ((i % 2) ? ratio : 1.0);
        GotoXY(scale * cos(angle), scale * sin(angle));
    }
} // End Star().


/////////////////////////////////////////////////////////////////////////////////
// StarSeries()
//
// Create a series of stars vith varying size and rotation.
/////////////////////////////////////////////////////////////////////////////////
void StarSeries()
{
    LOG_F(LOG_INFO, "StarSeries");

    // Generate some legal arguments for the calls to Star().
    uint_fast16_t points = random(MIN_STAR_POINTS + 2, MAX_STAR_POINTS / 4);
    double       ratio  = RandomFloat(MIN_STAR_RATIO, MAX_STAR_RATIO);
    uint_fast16_t size   = random(MIN_POLY_SIZE, (3 * MAX_SCALE_I / 4) + 1);
    double       rot    = RadAngle;

    // Determine how many steps to take, and how much to increase size and angle.
    uint_fast16_t steps   = 0;
    uint_fast16_t sizeInc = 0;
    double       rotInc  = 0.0;
    GenerateSeriesSteps(size, steps, sizeInc, rotInc);

    // Loop to create the Star series.
    for (uint_fast16_t i = 0; (i < steps) && !AbortShape; i++)
    {
        // Make the call to Star() and increment our size and rotation.
        Star(points, ratio, size, rot);
        size += sizeInc;
        rot += rotInc;
    }
    EndSeries();
} // End StarSeries().


/////////////////////////////////////////////////////////////////////////////////
// SuperStar()
//
// Create a star in which all outer points, with the possible exception of
// adjacent points (perimeter points) are connected via lines.
//
// Arguments:
//   - numNodes : Number of star points.
//   - size     : Size of the star.
//   - outline  : Set 'true' if the outline (perimeter) should be drawn.
//                Set to 'false' otherwise.
//   - rotation : How much to rotate the star, in radians.
/////////////////////////////////////////////////////////////////////////////////
void SuperStar(uint_fast16_t numNodes, uint_fast16_t size, bool outline,
               double rotation)
{
    // Make sure all arguments are within valid limits.
    numNodes      = constrain(numNodes, MIN_POLY_SIDES, 2 * MAX_POLY_SIDES);
    double scale = (double)constrain(size, MIN_POLY_SIZE, MAX_SCALE_I);
    double angle = rotation;

    // Show our call.
    LOG_F(LOG_INFO, "SuperStar(%d,%d,%d,%.1f)\n", numNodes, size, outline, RtoD(rotation));

    // If we are drawing a perimeter, then initial skip is 1.  Otherwise it is 2.
    uint_fast16_t initialSkip = (outline || (numNodes < 4)) ? 1 : 2;

    // Move to our start point.
    GotoXY(scale * cos(rotation), scale * sin(rotation));

    // Loop through all useful skip values.
    for (uint_fast16_t skip = initialSkip;
        (skip <= numNodes / 2) && !AbortShape; skip++)
    {
        LOG_F(LOG_DEBUG, "skip %d\n", skip);
        // Loop to visit each node with each skip value.
        for (uint_fast16_t nodesVisited = 0, startNode = 0;
             (nodesVisited < numNodes) && !AbortShape; startNode++)
        {
            uint_fast16_t node = startNode;
            LOG_F(LOG_DEBUG, "start node %d\n", node);
            // Loop to visit each node.
            do
            {
                LOG_F(LOG_DEBUG, "%d\n", node % numNodes);
                angle = rotation + (PI_X_2 * (double)node) / (double)numNodes;
                GotoXY(scale * cos(angle), scale * sin(angle));
                node += skip;
                nodesVisited++;
            } while (((node % numNodes) != startNode) && !AbortShape);

            // Return to the starting node.
            angle = rotation + (PI_X_2 * startNode) / (double)numNodes;
            GotoXY(scale * cos(angle), scale * sin(angle));

            // Make sure that if we are not displaying the outline that we don't
            // move between adjacent nodes.
            if (!outline &&
               ((nodesVisited < numNodes) ||
               ((nodesVisited == numNodes) && (startNode == 1))))
            {
                uint_fast16_t tempNode = (startNode + 3) % numNodes;
                LOG_F(LOG_DEBUG, "tempNode %d\n", tempNode);
                angle = rotation + (PI_X_2 * (double)tempNode) / (double)numNodes;
                GotoXY(scale * cos(angle), scale * sin(angle));
            }
        }

        // Return to the start node.
        GotoXY(scale * cos(rotation), scale * sin(rotation));
    }
} // End SuperStar().


/////////////////////////////////////////////////////////////////////////////////
// RandomSuperStar()
//
// Calls SuperStar() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomSuperStar()
{
    // Generate some legal arguments for the call to SuperStar().
    uint_fast16_t numPoints = random(MIN_POLY_SIDES, MAX_POLY_SIDES + 1);
    uint_fast16_t size      = random(MAX_SCALE_I / 2, MAX_SCALE_I + 1);
    bool          outline   = RandomBool();
    double       rot       = RadAngle;

    // Make the call to SuperStar().
    SuperStar(numPoints, size, outline, rot);
} // End RandomSuperStar().


/////////////////////////////////////////////////////////////////////////////////
// RandomShapes[]
//
// This is an array of ShapeInfo instances which is used to select random shapes
// to be displayed.
/////////////////////////////////////////////////////////////////////////////////
ShapeInfo RandomShapes[] =
{
    ShapeInfo(RandomRatios, 10),
    ShapeInfo(RandomSpirograph, 0),
    ShapeInfo(RandomSpirograph2, 0),
    ShapeInfo(RandomSpirographWithSquare, 10),
    ShapeInfo(RandomRose, 11),
    ShapeInfo(RandomClover, 11),
    ShapeInfo(RandomSuperStar, 25),
    ShapeInfo(RandomCircle, 15),
    ShapeInfo(RandomPlot, 20),
    ShapeInfo(PolygonSeries, 10),
    ShapeInfo(StarSeries, 10),
    ShapeInfo(HeartSeries, 15),
    ShapeInfo(EllipseSeries, 10),
    ShapeInfo(RandomRatiosRing, 10),
    ShapeInfo(RandomWipe, 30),
    ShapeInfo(RandomLines, 50)
}; // End RandomShapes[].


/////////////////////////////////////////////////////////////////////////////////
// GenerateRandomShape()
//
// Randomly selects a randomization function to execute.  The RandomShapes array
// contains a list of ShapeInfo instances that contain a pointer to a randomization
// function and its corresponding skip value.  It calls the corresponding
// ShapeInfo.MakeShape() method which determines whether or not it is ok to
// execute the corresponding randomization function based on the current iteration
// value.  If the shape function executes then MakeShape() returns a value
// of 'true'.  In this case, GenerateRandomShape() increments the iteration count.
// If the shape function does not execute, and returns 'false', the iteration
// count is not incremented, and another random shape is considered.
/////////////////////////////////////////////////////////////////////////////////
void GenerateRandomShape()
{
    // Keep track of our iteration.
    size_t index = 0;

    // Select functions to execute at random till we find one that will actually
    // execute.
    do
    {
        // Select a random function to execute.
        index = random(0, sizeof(RandomShapes) / sizeof(RandomShapes[0]));

    // Call the random function, and try another function if it was too soon.
    } while (!RandomShapes[index].MakeShape(ShapeIteration));

    // Increment the iteration count since we just executed something.
    ShapeIteration++;
    EndShape();
} // End GenerateRandomShape();


/////////////////////////////////////////////////////////////////////////////////
// ResetShapes()
//
// Resets (clears) the last cycle value of all shapes.  It is mainly used when
// a new random seed is set so that a previously generated sequence can be
// identically repeated.
/////////////////////////////////////////////////////////////////////////////////
void ResetShapes()
{
    for (size_t i = 0; i < sizeof(RandomShapes) / sizeof(RandomShapes[0]); i++)
    {
        RandomShapes[i].Reset();
    }
} // End ResetShapes().


/////////////////////////////////////////////////////////////////////////////////
// R E Q U I R E D   A R D U I N O   F U N C T I O N S
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// setup()
//
// This function initializes all the hardware, and homes the axes to (0, 0).
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

    // Initialize our serial port, wait for port to open, and issue a "Starting" message.
    Serial.begin(115200);
    delay(100);
    LOG_F(LOG_ALWAYS, "\r\nStarting\n");

    // Make sure we start with valid speed factors.
    RotSpeedFactor    = 1.0;
    InOutSpeedFactor  = 1.0;

    // Set the LEDs as early as possible.  They are at full brightness until
    // we read the pots and set their value for the first time.
    ReadPots();

    // Home the axes, then display status information.
    Home();
    Display();

    // Create an alarm for the rotational and in/out servo ISRs.  Start in 1 second.
    add_alarm_in_us(1000000, RotaryServoIsr, NULL, false);
    add_alarm_in_us(1000000, InOutServoIsr,  NULL, false);

    // Seed the random number generator.
    RandomSeed = get_rand_32();
    randomSeed(RandomSeed);

    // Display the random number.  It may be used in the future to repeat an
    // interesting sequence.
    LOG_F(LOG_ALWAYS, "Seed = %ul\n", RandomSeed);

    // Initialize runtime variables to safe values.
    SpeedDelay        = SPEED_DELAY_MAX_VAL;
    RemoteSpeedDelay  = false;
    Brightness        = BRIGHTNESS_MIN_VAL;
    RemoteBrightness  = false;
    RemotePause       = false;
    AbortShape        = false;
    ShapeIteration    = 0;
    RandomSeedChanged = false;
    ResetShapes();

    // Announce that we are ready to go.
    LOG_F(LOG_ALWAYS, "\nREADY\n");
} // End setup().


/////////////////////////////////////////////////////////////////////////////////
// loop()
//
// This is the main system loop.  Random shapes are displayed via this function.
/////////////////////////////////////////////////////////////////////////////////
void loop()
{
    // On first iteration after boot, we wipe the board and display my initials.
    static bool firstTime = true;
    if (firstTime)
    {
        // Wipe the board.
        ClearFromIn();

        // Change this as desired.  It is the power-up greeting.
        RotateToAngle(atan2((double)JMCPlot[0].y, (double)JMCPlot[0].x));

        // Show JMC...
        PlotShapeArray(JMCPlot, sizeof(JMCPlot) / sizeof(JMCPlot[0]), false,
                       "JMCPlot");
        EndShape();
        firstTime = false;
    }

    // Generate a random shape.
    GenerateRandomShape();
    ReadPots();

    // If we were aborting the previous shape, we're done now.
    AbortShape = false;

    // Start over with clean board if the random seed has changed.
    if (RandomSeedChanged)
    {
        ResetShapes();
        Home();
        ClearFromIn();
        RandomSeedChanged = false;
    }

    delay(10);
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
