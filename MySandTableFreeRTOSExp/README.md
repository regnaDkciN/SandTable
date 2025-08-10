Upgrading RP2350 Based Sand Table to FreeRTOS
===
![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)


This document describes the firmware design and code changes that were made to the [RP2350 FreeRTOS based Sand Table](https://github.com/regnaDkciN/SandTable/tree/main/MySandTableFreeRTOS) to queue motion moves ahead of execution.  Major changes also include an improved method of randomly selecting the next shape to draw based on weighted probabilities.  The new version -  [MySandTableFreeRTOSExp](https://github.com/regnaDkciN/SandTable/tree/main/MySandTableFreeRTOSExp) - was so named because it was initially an experimental version, but has become stable and the name just stuck.

## Motivation
The FreeRTOS version of the sand table software went a long way to improve motion smoothness.  However, some motion glitches were still possible due to the time it takes to calculate the next move.  This new version adds pre-execution motion planning and queueing which goes further in removing motion glitches. 


## The Problem
As explained in the MySandTableFreeRTOS version, move calculations can take up enough time to cause glitches as seen in the timing diagram below:

<img src=https://i.imgur.com/yZ1QpDW.png width=1024>

The important points to note are:
* The timing starts with motion ISRs idle and the Shape Task generating the next move.
* At time 30, the Shape Task has completed generating the next target position and starts to pend on motion completion (i.e. it goes to sleep).
* At time 40, the ISRs see that new motion is needed, and starts moving the axes.
* At time 88, the move completes, and the ISRs notify the Shape Task.  The shape task wakes up as soon as the ISR completes.
* The Shape Task begins generating the next move.
* At time 110, the Shape Task completes its next target position calculations and starts the next move.  It then starts to pend on motion completion again.
* The cycle repeats after this.
* A motion glitch occurs when the ISRs are idle at time 100.

If motion generation calculations could be calculated ahead of time, this glitch could be removed.

## New Design
The diagram below attempts to show the most important parts of the old design's motion related code structure:

<img src=https://i.imgur.com/18xk1SS.jpeg width=800>

Execution proceeds as follows:
1. A shape function calculates the next X,Y position of the shape and calls ```GotoXY()`` with the new position.
2. ```GotoXY()``` loops, breaking the move into smaller segments, basically interpolating between the current position and the new segment position.  It calls ```MoveTo()``` with the new segment position.
3. ```MoveTo()``` calls ```ReverseKinematics()``` with the new segment position, then goes to sleep waiting on the move to complete.
4. ```ReverseKinematics()``` generates the servo positions corresponding to the new segment X,Y coordinates.  It then starts the move and returns to ```MoveTo()```.
5. The ISRs repeatedly execute, moving the axes to the new position.  When the move completes, the ISRs signal the Shape Task which wakes up, completing ```MoveTo()```'s execution.
6. ```MoveTo()``` returns to ```GotoXY()```, which returns to the original shape function, and the process repeats.


In order to improve the motion performance, the Shape Task has been split into two tasks and a new Motion Queue has been added:

<img src=https://i.imgur.com/uwbCvJU.jpeg width=1024>

* The new Shape Task contains the code to calculate moves and queues the new position data to the new Motion Queuel Task.  The Shape Task still executes on core 0 at priority 5.
* The new Servo Control Task takes moves off the Motion Queue and sends them to the servo ISRs for execution.  It executes on core 0 at priority 6.
* The new Motion Queue holds a limited number of ```PlannerData``` structures.  The ```PlannerData``` structure holds the target steps for each servo:
	```
	// PlannerData structure.  Used to pass data between the planner and the
	// servo controller task.
	struct PlannerData
	{
	    int_fast32_t m_InOutStepsTo;    // In/out target steps for move.
	    int_fast32_t m_RotStepsTo;      // Rotary target steps for move.
	};
	
	```
	
*Note that ```MotorRatios()``` moves are handled differently from normal moves since they are not really planned, and are mainly handled in the ISRs.  The Servo Control Task does not participate in ```MotorRatios()``` moves.*

To help further understand the new design, compare the following new design diagram to the old design diagram above:

<img src=https://i.imgur.com/tJWiD7c.jpeg width=600>

Execution of the new design proceeds as follows:
1. Just as before, a shape function calculates the next X,Y position of the shape and calls ```GotoXY()`` with the new position.
2. Just as before, ```GotoXY()``` breaks the move into smaller segments, basically interpolating between the current position and the new segment position. However, unlike before, ```GotoXY()``` calls ```ReverseKinematics()``` with the new segment position.
3. ```ReverseKinematics()``` generates the servo position based on the new segment X,Y position.  It then attempts to send the new servo position to the Motion Queue via a call to ```xQueueSend()```.  If the Motion Queue is already full, the task goes to sleep waiting for space to free up on the Motion Queue. 
4. When space becomes available on the Motion Queue, ```ReverseKinematics()``` saves the new servo position on the queue and immediately returns to ```GotoXY()```.  
5. Concurrently with items 1 through 4, the Servo Control Task is waiting for the Motion Queue to become non-empty via a call to ```xQueueReceive()```.
6. Once a new servo position is fetched from the Motion Queue, the Servo Control Task starts the move and begins waiting for the move to complete (it goes to sleep waiting) via a call to ```xTaskNotifyWait()```.
7. As soon as motion completes, the ISRs wake the waiting Servo Control Task via a call to ```xTaskNotifyFromISR()```.
8. The process repeats.

Note that this new design allows several moves to be calculated ahead of time, which allows the Servo Control Task start new moves immediately after completion of the preceding move.  This minimizes the possibility of generating a motion glitch due to lengthy  position calculations.


## Other Motion Related Code Changes

### New Variables
Several new global variables were needed.  These include task and queue handles and new working versions of some of the motion variables.  The new variables include:
```
// RTOS related variables.
TaskHandle_t      ServoCtrlHandle    = NULL;    // Handle for servo control task.
TaskHandle_t      ShapeHandle        = NULL;    // Handle for shape task.
QueueHandle_t     PlannerQueueHandle = 0;       // Handle for planner queue.
const UBaseType_t PLANNER_Q_LENGTH   = 20;      // Num entries in planner queue.
volatile float_t  PlannerCurrentX    = 0.0;     // Planned location of ball (X).
volatile float_t  PlannerCurrentY    = 0.0;     // Planned location of ball (Y).
volatile bool     MoveInProcess      = false;   // True if moves are in process.
```

### GotoXY()
The new version of ```GotoXY()``` uses working values for the current X,Y position rather than actual current values:
```
volatile float_t  PlannerCurrentX;     // Planned location of ball (X).
volatile float_t  PlannerCurrentY;     // Planned location of ball (Y).
```

 These working values are based on the final position of the previously queued move.  This way, ```GotoXY()``` can plan ahead based on the expected starting position instead of the current actual position.  In order to keep the working values current, they are updated in ```StartShape()```, which is called before starting each shape position calculation.  Just to be safe, they are also updated at the end of each shape, after all movement has completed, in ```EndShape()```.
 

### HandleRemoteCommandsTask() Changes
The commands that cause the current motion to abort ('R' and 'N') added code to reset the Motion Queue via a call to ```xQueueReset()```.  This insures that the current motion will stop immediately.

### RotateToAngle()
The ```RotateToAngle()``` function rotates the rotary axis to a specified angle from the origin leaving the in/out arm position unchanged.  Unlike ```ForceInOut()``` and ```ForceRot()```, this function generates a coordinated move.  However, it is more like ```MotorRatios()``` in that it bypasses

### WaitForMoveComplete()
A new function - ```WaitForMoveComplete()``` was added to wait for the last move of a shape to complete before continuing on.  This was necessary due to the addition of the Motion Queue.  Shapes must fully complete execution, including finishing all queued moves, before the shape is considered to be complete.  ```WaitForMoveComplete()``` is called from a few places in the code that are not strictly considered as being the end of a shape.  These places include any place that ```GotoXY()``` is called, but not as part of a shape.  For example, the code for ```ExtendInOut()`` is now the following:
```
void ExtendInOut()
{
    GotoXY(MAX_SCALE_F * cosf(RadAngle), MAX_SCALE_F * sinf(RadAngle));
    WaitForMoveComplete();
} // End ExtendInOut().
```



## Weighted Random Numbers
This shape selection logic was replaced by a new class - ```RandomVoseAlias``` - that provides for efficient sampling of random values from a discrete probability distribution.  For a complete writeup on the alias method, including the intuition and important proofs, please see the article ["Darts, Dice, and Coins: Sampling from a Discrete Distribution"](http://www.keithschwarz.com/darts-dice-coins/).

### ShapeInfo Class Changes
The old version of the sand table software used a clunky means of selecting the next random shape.  It would tentatively select a random shape, then check to see if the shape had been executed recently.  If so, it would select a new random shape, and repeat the process until a suitable shape was found.  The ```ShapeInfo``` class contained a ```m_Delay``` value that was used to determine the number of cycles that must elapse between consecutive generation of each shape was allowed.  The class also contained logic to remember the cycle at which the last execution of each shape took place.

The new version replaces the ```m_Delay``` member with the ```m_Weight``` member.  It represents the probabilistic weight of this object.  It is used to generate a weighted probabilities vector which is used to randomly select shapes for execution.  A higher value causes the shape to be selected more frequently.  For more info see the Random Shapes section below.

The ```MakeShape()``` method also changed to simply call the associated shape function, and keep some statistics.

### RandomShapes[] Table
The weight value of each entry of the ```RandomShapes[]``` table is  based on how often each shape should execute based on how often a ```RandomWipe``` should be executed.  For example the code below shows that:
- ```RandomPlot``` should execute 2 times as often as ```RandomWipe```.
- ```RandomLines``` should execute 3 times as often as ```RandomWipe```.
- ```RandomSuperStar``` should execute 10 times as often as ```RandomWipe```.
- And so on...
```
ShapeInfo RandomShapes[] =
{
    ShapeInfo("RandomWipe", RandomWipe, 1),
    ShapeInfo("RandomPlot", RandomPlot, 2),
    ShapeInfo("RandomLines", RandomLines, 3),
    ShapeInfo("RandomSuperStar", RandomSuperStar, 10),
    ShapeInfo("EllipseSeries", EllipseSeries, 10),
    ShapeInfo("RandomCircle", RandomCircle, 12),
    ShapeInfo("HeartSeries", HeartSeries, 15),
    ShapeInfo("RandomSpirographWithSquare", RandomSpirographWithSquare, 20),
    ShapeInfo("RandomSpirograph2", RandomSpirograph2, 25),
    ShapeInfo("RandomRose", RandomRose, 25),
    ShapeInfo("RandomClover", RandomClover, 25),
    ShapeInfo("RandomRatios", RandomRatios, 30),
    ShapeInfo("RandomSpirograph", RandomSpirograph, 30),
    ShapeInfo("PolygonSeries", PolygonSeries, 30),
    ShapeInfo("StarSeries", StarSeries, 30),
    ShapeInfo("RandomRatiosRing", RandomRatiosRing, 30)
}; // End RandomShapes[].
```

### InitProbabilities()
A new function - ```InitProbabilities()``` -  was added to initialize the weighted probabilities for each entry of the ```RandomShapes[]``` table.  It steps through the entries of the ```RandomShapes[]``` table and sums their weights.  It then creates a probability vector of associated shape probability via dividing each entry weight by the total weight.  It then initializes the weighted probability random shape generator via a call to ``` WeightedRandom.Init()```.  Global variables used for weighted random shape generation are:
```
// Create a weighted random object for use in selecting the next shape to execute.
RandomVoseAlias WeightedRandom;
// Vector of weighted probabilities used with WeightedRandom.
std::vector<float_t> Probabilities;
```


### GenerateRandomShape()
The ```GenerateRandomShape()``` function was simplified to select a new random shape and execute it. Its has simplified to the following:
```
void GenerateRandomShape()
{
    // Execute a random shape.
    RandomShapes[WeightedRandom.Next()].MakeShape(ShapeIteration);

    // Increment the iteration count since we just executed something.
    ShapeIteration++;
} // End GenerateRandomShape();
```

