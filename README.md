# Sand Table
![Sand Table](https://i.imgur.com/tNOBlu8.jpeg)
I have always been interested in sand tables, but most required more carpentry work than I could handle, and were too expensive for my budget.  Then I came across  the Instructable of [<u>World's First Cycloid Art Table: How I Built This Arduino-Powered Spirograph Machine</u> by NewsonsElectronics](https://www.instructables.com/Worlds-First-Cycloid-Art-Table-How-I-Built-This-Ar/).  This was finally a reasonable design that I could handle.

My background is in real time embedded firmware, so the software wasn't a problem.  Woodworking is pretty much out of my league.  However, this project was mainly laser cut, and my local library had a laser cutter available, so I learned how to use it with pretty good results.  Along the way I made quite a few changes to the design, and am pretty happy with the outcome.

This site describes some of the modifications I made on the way. Notable changes include:
* Addition of a home limit sensor with corresponding firmware.
* Addition of a 3D printed Arduino bumper for mounting the Arduino UNO board to the base.
* Addition of a top ring to hold a plexiglass window.
* Greatly modified Arduino sketch which adds many new shapes, randomization functions, and fixes several problems that were encountered with the original sketch.
* Addition of a power switch with associated 3D printable box.
* Replacement of the small wooden gears with new 3D printed ones.
* The highly modified original software version for use with the Arduino UNO - *MySandTable*.
* A software version - *MySandTable2350* - that replaces the Arduino UNO with an Adafruit Metro RP2350 board.
* A software version - *MySandTableFreeRTOS* - that uses the FreeRTOS real-time operating system on the Metro RP2350 board.
* A software version - *MySandTableFreeRTOSExp* - that plans motion several moves ahead in order to help smooth out the motion.  This version was originally experimental, thus the 'Exp' suffix.  However, it is currently stable and is the preferred RP2350 version.
---
# General Issues/Fixes/Useful Information

This was my first experience with laser cutters.  The Instructable included .DXF and .SKP files for the laser cut parts.  I couldn't make use of the .SKP file, but found that I could work with the .DXF files.  For these I used [QCAD](https://qcad.org/en/) which is a free, open source application for 2D CAD drafting.  The available laser cutter required .PDF files though.  I ended up using QCAD to edit the original .DXF file and produce a .SVG file.  Then I used [Inkskape](https://inkscape.org/?about-screen=1) to convert the .SVG file to .PDF.  The workflow was as follows:
1. Use QCAD to edit the original .DXF file.
2. Use QCAD to generate the .SVG file.
2. Use Inkscape to add text and other features to the .SVG file.
3. Use Inkscape to change cut lines to 0.1pt width (required by the laser cutter).
4. Use Inkscape to create the .PDF file which is used by the laser cutter.

## Wood Selection
Used Baltic Birch Plywood- B/BB Grade 1/8" / 3mm / 24"x24"  (https://makerstock.com/).  Liked the quality of the wood and the price was reasonable.

## Steel Ball
Instructable showed both 15mm ball and 8mm ball.  I experimented and ended up replacing the steel ball with stack of 4 each [3mm x 2mm magnets](https://www.amazon.com/dp/B09SLFSRBP).

## Sand
I originally used the recommended sand in the table but found it to cause jerky ball motion, and was pretty noisy. After an extensive web search, found  that [Shuffleboard Sand](https://www.amazon.com/dp/B08NFZFGQF) was an excellent replacement.  This produces much smoother and quieter movement.

## Magnets
The Instructable used a single 15mm x 3mm magnet on the end of the linear arm.  I stacked 2 each 15mm x 3mm magnets for better ball following: (https://www.amazon.com/dp/B0B5G6XS4J).  At one point I tried 3 stacked magnets, but found that 2 worked just as well.

## Miscellaneous Table Construction
* I found that the motors didn't originally line up vertically.  This may have been due to my motors not being sized exactly as the specified ones, or due to my lack of construction skills.  At any rate, I ended up with 3 spacers under the center (linear) motor, and none above it.  I also used 1 spacer below the outer (rotary) motor and 2 spacers above it.  Due to this, the small pillar needed to be modified slightly to fit correctly in my setup.  I cut 3mm from lip that goes over servo to account for extra spacer below.
![Modified Small Pillar](https://i.imgur.com/EJCw9XU.png)
* The Instructable wasn't very clear about the number of small rings needed to hold up the table.  I used 8 layers of small rings, which also accounts for the extra magnet that I used.  Similarly, it wasn't clear how many large rings should be used on the table.  I settled on 5 large rings because I wasn't sure about the ball size.  However, after deciding on using 4 each of 3mm x 2mm magnets in place of a steel ball, it is clear that I could have used 4 rings instead.
![Rings Callout](https://i.imgur.com/u8Uh9qX.jpeg)
* After assembly, I found that the table wasn't securely attached to the base, so it was too easy to attempt to pick the unit up and have the table detach from the base.  I did not want the table to permanently connect to the base since I knew that maintenance would then be impossible.  I decided on the simple approach of drilling  1/16" holes in the table vertical rods, and insert a wire through the hole to keep the table attached to the base.  This allowed for easy  detachment of the table from the base by simply removing the wires from the rods.
![Retaining Wire](https://i.imgur.com/207iJ32.jpeg)
* The original holes for the potentiometers were too close to the Arduino UNO board.  I moved them to the center of the corresponding boards.  I also added labels as shown below.  After some use, I decided that the pot labels would be better placed above the potentioneters, but left them as is.
 ![Front Low Level View](https://i.imgur.com/gy55XrQ.jpeg)
* I found that the potentiometer face plates were not very secure.  To fix this, I carefully drilled 5/64" holes through the faceplates and base plate and inserted small nails.
* I originally added rubber feet to the bottom of the base.  It turned out that they prevented easy shaking of the table to clear it, which is very useful when debugging.  I replaced the rubber feet with felt pads and found that these were much better.  The feet I ended up using were [these](https://www.amazon.com/Black-Self-Adhesive-Felt-Bumpers/dp/B07DYQJRDB/ref=sr_1_1?sr=8-1).
* I have added a 3D printed part - "Bezel3.stl" - to strengthen the face plate panels (the ones that hold the pots and the UNO face plate).  It contains a channel that the top of the face plate panels seat into and provides some rigidity.  It also helps to protect the wiring of the electronic parts.
![Bezel](https://i.imgur.com/hG6tkcU.jpeg)

----

# GEARED PARTS
I had several problems with the gears.  This section contains information regarding my findings and fixes.

## Dowel Rods
I couldn't find the tapered dowel rods that were recommended in the Instructable write-up.  I ended up using [1/8" wooden dowel rods](https://www.amazon.com/dp/B0CXDDGRFK) for the geared parts .  This required changing all the dowel rod holes in the geared parts to 3 mm.  The "Modified Parts" files contain the modified parts.  Note that two of the large gear are required.

## Bearing
I couldn't find recommended bearing.  Instead I used a [F695-2RS Bearings, 5x13x4mm Ball Bearing](https://www.amazon.com/dp/B0CZ6RBJSZ) which worked out well.

## Reversed Linear Gear
I accidentally mounted the linear gear in the opposite direction from how the Instructable  video shows it.  This required several modifications to the Arduino sketch.  If you use my program, you may want to flip the linear gear as I did to avoid problems with my code.
![Flipped Linear Gear](https://i.imgur.com/Q2l8WMY.jpeg)

## Large Rotary Gear
* The Instructable was ambiguous regarding the large rotary gear.  I found that it should be doubled up (i.e. one on top of the other) and in my case I needed the center hole diameter to be 12.7mm.  The included  "Modified Parts" files have have been updated with these changes.  Note that two of the large gear are required.
* Neither of the laser cut small gears worked well.  Both were a little too wide, probably due to a difference in kerf width between the laser cutter I used and the one used in the Instructable.  I experimented and came up with 3D printed gears to replace both of them.
* ![Doubled Up Large Gear](https://i.imgur.com/ccbu8Gc.jpeg)

## Small Gears
* The small outside (rotary axis) gear needed to be at least 5mm thick, which is OK since the large rotary gear is doubled up.  The included **Small Gear Thicker.stl** file can be used to 3D print  this thicker gear.  I used PLA for the gear with 30% infill and 0.2mm layer.  The PLA gear has worked great with over 100 hours of run time, but I would guess that ABS or PETG would be better.
* The small center (linear) gear did not catch the motor shaft enough and gave constant problems.  I created a new 3D printed gear which adds a 2.4mm long 11mm diameter shaft which fits in the center hole of the large gear, and gives more area of contact between the small gear and the motor shaft.  The included **Small Gear With Bushing.stl** file contains the result. I used PLA for the gear with 30% infill and 0.2mm layer.  The PLA gear has worked great with over 100 hours of run time, but I would guess that ABS or PETG would be better.

# Motors
* As previously mentioned, I had problems with motor height alignment.  I ended up with 3 spacers under the
  center motor, and none above.  Used 1 spacer below the outer motor, and 2 above.
  ![Rotary Motor Spacers](https://i.imgur.com/3L1soeO.jpeg)
  ![Linear Motor Spacers](https://i.imgur.com/TjD4wYw.jpeg)
 
* I used these [Nema 17 stepper motors (17HS4023)](https://www.amazon.com/dp/B0C36D34F5).  Their original cables were too long which made it hard to stow them on the base.  I replaced the original motor cables with shorter [(500mm) stepper cables](https://www.amazon.com/dp/B07Q12B6K5?ref_=ppx_hzod_title_dt_b_fed_asin_title_0_0).  These solved the size problem, but weren't wired the same as the original cables.  In order to make them work, the 2 middle conductors needed to be swapped.  I just used brute force and pulled them out of on end, swapped them, then pushed them back in.  This worked OK and I have had no problems with them.

* The screws from the original motors were too short.  I replaced the center motor screws with m3 x 6mm, and the outer motor screws with m3 x 14mm.

* The holes in the motor spacers were too small.  I enlarged them using a 7/64" bit, which worked out well.

# CNC Shield
* I used the recommended [Arduino CNC Shield](https://www.amazon.com/dp/B08KFYKKN4).  In bringing it up I had several major issues.  Some were my fault, and some were due to the broken motor drivers that were included with the shield.  In the end, these were the problems and workarounds:
	+ Not one of the 4 included stepper drivers functioned at all.  In fact, one of the drivers caused the CNC shield's fuse to blow, and blew a trace to the X-Axis.  The Instructable recommended the use of other drivers, and I bought them, but I wanted to get it to work with the original drivers before I switched to the good ones.  I should have just started with the [TMC2209 Stepper Drivers]( https://www.amazon.com/dp/B08SMDY3SQ), and would recommend everyone do the same.  I ended up bypassing the fuse, and using the shield's Y- and Z-Axis instead of the X- and Y-Axis.  The Y-Axis now controls the linear in/out axis, and the Z-Axis now controls the rotary axis.
	+ The Instructables author also contributed two enhancements for the CNC shield which I applied to my setup. Neither of these enhancements is absolutely necessary, but I think they are useful.  Both of the modifications can be found at the Instructables site as [Arduino CNC Shield V3 - Enhanced Capabilities and Features](https://www.instructables.com/Arduino-CNC-Shield-V3-Enhanced-Capabilities-and-Fe/).  I modified the shield to supply power to the Arduino, and to add the ability to enable the axes individually (based on Y- and Z-Axes).

# Stepper Drivers
* The DRV8825 stepper drivers that came with the CNC shield were total junk.  Not a single one of them worked.  Trying to use one of them resulted in blowing the CNC shield fuse, and damaging the X-Axis socket.
* As recommended, I used [TMC2209 Stepper Drivers]( https://www.amazon.com/dp/B08SMDY3SQ) instead of the ones that came with the CNC shield.
* Setting the motor current of the TMC2209 stepper driver is well explained [HERE](https://www.youtube.com/watch?v=VcyGzXIZm58).  The desired current for these motors is less than 700mA per phase.

  		Vref = Desired Current x 0.71 x 2 
  		     = 0.99V for 700mA per phase
  		
# Arduino UNO
* I used the Arduino UNO that came with the CNC shield.
* Found that mounting the Arduino is difficult, especially when gear assembly is already in place.  Created a 3D printed Arduino bumper with side mounting holes to help in placing the Arduino.  See the included **Arduino Bumper.stl** file.
* Found that M3 screws fit nicely into 7/64" holes in plywood.  Used M3 screws to mount the Arduino with bumper to base plywood.
![Arduino Mount](https://i.imgur.com/OUUaUbT.jpeg)

# Power Box
* I used this [Power supply: 12V 8A](https://www.amazon.com/dp/B0CL1RX1TX).  It is definitely overkill, but works well and was not too expensive.
* I wanted a clean way to connect power to the sand table, so I decided to use a 3D printed power box with a power connector that matched the power supply, and a switch.  The switch box with lid is included as **Power Box.stl** and **Power Box Lid.stl**.  Used PLA, 20% infill, and 0.2mm layer height for the print.
![Power Box 1](https://i.imgur.com/hZSH33R.jpeg)
![Power Box 2](https://i.imgur.com/NhfUZde.jpeg)
* I used this [Power supply connector](https://www.amazon.com/dp/B0D9JGXBJB).
* I used a switch that I had on hand, similar to [this one](https://www.amazon.com/uxcell-Position-Button-Rocker-Switch/dp/B0727PBJLV).
* Drilled two 3/16" holes through the base of the power box and the base of the sand table for mounting.  Used m3 x 5mm screws to secure the power box to the base.

# Rotary Home Sensor
* I wanted to be able to home the axes to a known starting position so that things like text could be displayed in a consistent manner.  So I added a home limit reed switch using GPIO 5.  This GPIO pin was originally the X axis direction pin, but since I am using Y and Z, the X axis direction pin was available.  Here's an example of some text that will always print upright due to the axes being homed.
![JMC](https://i.imgur.com/kQomHa9.jpeg)
* I used [Reed Switch Reed Contact Normally Open (N/O) Magnetic Induction Switch (2mm*14mm)](https://www.amazon.com/dp/B07RS2M9TR) to sense the pointer magnet.  There was no need to add any additional magnets.
* I created and 3D printed a mount for the reed switch, which is included as **Reed Switch Mount.stl** and mounted it near the Brightness pot. Used PLA, 15% infill, and 0.2mm layer height for the print.  Used m3 x 4mm screws to secure the mount to the horizontal member.   Drilled a 3/16" hole in the horizontal member near the mount to pass the sensor wires through to the Arduino.  Used 
![Home Sensor Mount](https://i.imgur.com/x8Liq3m.jpeg)
* Updated the Arduino sketch Home() function to use new home limit switch.  See the .ino file.
* The Home() operation behaves as follows:
    ```
 	   Extend the in/out axis all the way out
	   If already on the home sensor
	       Then rotate CCW till off of the sensor
	   Rotate CW till the home sensor is detected
	   Rotate CCW by the amount specified by HOME_ROT_OFFSET
	   Retract the in/out axis all the way in
	   Initialize all position and motor variables to position (0,0)
	   Enable both motors
* Note that an offset may be applied (HOME_ROT_OFFSET) in order to align the rotation of the origin as desired.

# Miscellaneous Electronics
## Potentiometers
* I used these [10K pots](https://www.amazon.com/dp/B0BRGX2FTL).

## LEDs
* I use these [LEDs](https://www.amazon.com/gp/product/B0C8HTH282)
* Drilled a 5/32" horizontal hole into the upper ring to route LED cable into the ring vertical hole.
* Drilled a 5/16" hole in vertical support to pass LED cable through.
* Added a  [2 Conductor JST Connector](https://www.davesrce.com/product-page/jst-connectors?gQT=1) to attach the LEDs to the Arduino UNO so that top may easily be disconnected from base.
* Added a [Self-Adhesive Super Klip](https://www.amazon.com/Super-Glue-KW10-12-Klips-120-Pack/dp/B00FY8WHEU) as a cable organizer for the LED cable.
![LED Connector 1](https://i.imgur.com/DXwuwZe.jpeg)
![LED Connector 2](https://i.imgur.com/6Vc9H8k.jpeg)

## MOSFET 
* Used [Power MOSFET IRFZ44N](https://www.amazon.com/dp/B0CHQ8QJ5K).  Note that the parts list on the Instructable is incorrect and specifies BD139N transistor when IRFZ40N should be used.  IRFZ44N is an acceptable replacement.
* Added a 100K resistor between the MOSFET gate and ground (source) to keep the LEDs from turning on full brightness at power-up.
* Used metal 3mm x 8mm bolt to secure MOSFET to left Arduino mounting wing through hole drilled in base.  Also use 2  m3 washers for spacing and heat dissipation.
![MOSFET Mount](https://i.imgur.com/AMYUQlH.jpeg)

# Window
* Created a top outer ring that is used to hold the acrylic window.  The attached **Window Holder no Text.pdf** file corresponds to this window holder.
* Used [12" x 12" Clear Cast Acrylic Sheet]( https://www.amazon.com/dp/B0BBQ8B41F) for the window.  Cut it circular with laser the cutter using the attached **Window.pdf** file.  It fits inside top Window Holder ring.
* Found that the acrylic widnow generates enough static to attract the Shuffle Board Sand.  I don't know if real sand would have the same problem, but the shuffle board sand really did.  I was a little skeptical, but decided to try an [Anti-static CRT Cleaner](https://www.amazon.com/dp/B00UM9C04S?ref_=pe_123509780_1038749300_fed_asin_title) and found that it worked well and completely eliminated the problem with sand clinging to the acrylic window.  I would recommend its use.
* Generated simple 3D printed cylinders (**Window Knob.stl**) to use as knobs for the window.  Used PLA, 20%infill, and 0.2mm layer height for the print.  Attached them to the window with glue.
![Window Knob](https://i.imgur.com/bxLPUnK.jpeg)

# Software

The included Arduino sketch (**MySandTable.ino**) was based on the [Instructable]( https://www.instructables.com/Worlds-First-Cycloid-Art-Table-How-I-Built-This-Ar/) Arduino sketch **working10iteration.ino**.  It was, however, almost completely rewritten to add features and make it more understandable.  A few of the changes were:
* Re-formatted, including renaming variables and functions, using consistent indenting, and generally cleaning up the code.
* Uses Y- and Z-Zxes of the CNC shield instead of X- and Y- of the original.  This was due to the X-axis hardware on my CNC shield being damaged.
* Used some slightly different algorithms in a several spots.
* Added several more shapes, and randomization functions for each shape.
* Many more changes to fix anomalous behavior and enhance operation.

## Serial Logging
The **SerialLog.h** file was added to help with making groups of data logging to be easily enabled or disabled.  It contains macros to selectively print information based on some user-defined level macros.
Prints will only generate output if the corresponding macro is set to `true`.  `SerialLog.h` should be placed in the same directory as the Arduino sketch (`MySandTable.ino`).

Logging macros include:
* `LOGF(pBuf, bufSize, level, ...)` <br>
	This macro implements a formatted print.  It uses a printf style format to create a string to print.  The limitation of this macro is that it cannot print 32-bit integers or floats.  Its arguments are:<br>
&nbsp;&nbsp;&nbsp;&nbsp;`pBuf` : A pointer to a buffer to hold the formatted output data.<br>
&nbsp;&nbsp;&nbsp;&nbsp;`bufSize`: The size, in bytes, of `pBuf`.<br>
&nbsp;&nbsp;&nbsp;&nbsp;`level` : A value that will be evaluated as either `true` or `false`. If `true`, then the printf style string will be sent out the serial port.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Otherwise no action will be taken.<br>
&nbsp;&nbsp;&nbsp;&nbsp;`args`: This is a printf style string with optional additional arguments.
* `LOGU(level, v)`<br>
	This macro implements an unformatted print.  It simply prints the given single value as long as `level` evaluates to `true`.  Its arguments are:<br>
 &nbsp;&nbsp;&nbsp;&nbsp;`level` : A value that will be evaluated as either `true` or `false`.  If `true`, then the value will be sent out the serial port.
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Otherwise no action will be taken.<br>
&nbsp;&nbsp;&nbsp;&nbsp;`v`: This is the value that will printed if `level` evaluates to `true`.  It can be any valid printable data type.

The top of MySandTable.ino contains macros to enable/disable specific types of information to be sent via the serial port as well as macros to simplify logging macros use.
```
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
#define LOG_BUF_SIZE     60         // Important: size of formatted log buffer.
                                    // All formatted log strings are limited to
                                    // this size.
char LogBuf[LOG_BUF_SIZE];          // Buffer for formatting log data.

// Logging macros to ease log use.
#define LOG_F(level, ...)   LOGF(LogBuf, LOG_BUF_SIZE, level, __VA_ARGS__)
#define LOG_U(level, v)     LOGU(level, v)

```

* `LOG_DEBUG` - Setting this macro `true` will enable debug related information to be sent via the serial port.  This was mainly used to debug the SuperStar shape, and is normally set to `false` to disable sending its data.
* `LOG_VERBOSE` - Setting this macro `true` will enable verbose information to be sent via the serial port.  This was mainly used in early debugging of the movement generation code, and is normally set to `false` to disable sending its data.
* `LOG_INFO` - Setting this macro to `true` will enable informational data to be sent via the serial port.  The data sent by enabling this macro describes the call to the shape that is currently executing.  It includes the name of the function being executed, and a list of all of its arguments.  This macro is normally set to `true`. (A possible future project will use a Raspberry Pi Zero W to read this information, and save it along with a picture of the resulting table state.  Stay tuned.)
* `LOG_STEP`- Setting this macro `true` will enable the ability to single step the plotting of shape array data.  This can be useful for optimizing the memory footprint of any newly added shape array.  For example, by single stepping through a shape array, one may identify redundant or useless moves that may be removed.  This macro is normally set to `false`.
* `LOG_CYCLES`- Setting this macro `true` will generate cycle countdown information.  Certain shapes loop through multiple cycles before they complete.  Enabling this macro displays a countdown of the number of cycles remaining for the particular shape.  This macro is normally set to `false`, but if you like to have an idea of how much time is left when generating a shape, then set it to `true`.
* `LOG_ALWAYS` - This macro should always be set to `true`.  All data that should be sent unconditionally uses this macro.

A buffer that is used for formatting log data is also created here.  The buffer, `LogBuf[LOG_BUF_SIZE]`, must be sized large enough to handle the largest formatted log message.  `LOG_BUF_SIZE` is initially set to 60, which should be enough to handle all existing formatted log messages.

Two more macros (`LOG_F` and `LOG_U`) are also created here to make using the serial logger easier.  All logging done in MySandTable.ino uses one of these two macros.
 
 ## User Settable Constants
 The following macros are meant to be settable by the user depending needs.
 ```
 #define PAUSE_ON_DONE  false // Set to 'true' to pause when drawing finishes.
#define ROT_CAUSING_IN CCW   // Set to 'CW' if clockwise rotation causes IN movement.
                              // Set to 'CCW' if counter clockwise rotation causes IN movement.
```
 
* `PAUSE_ON_DONE` - This macro is used while debugging/testing.  If `true`, then execution will stop after each shape is drawn.  Execution will begin only after the Speed pot is turned fully CCW (the pause position), then  moved off of the pause position.<br>
* `ROT_CAUSING_IN` - Due to the mechanical configuration of the sand table, movement of the rotary axis causes a proportional move of the linear axis based on the direction of rotary rotation.  The firmware compensates for this movement.  In order for the compensation to function properly, the firmware must know which direction the linear axis moves in relation to the rotary axis.  (Due to an oversight on my part, my linear gear behaves opposite of the one in the original Instructable, thus needing attention).  This macro lets the firmware know which direction of rotary axis movement will cause the linear axis to move inward (toward zero).  Values for this macro should be either `CW` for clockwise, or `CCW` for counter-clockwise.  I believe the original Instructable hardware would use `CW`.  Since mine is reversed, I use `CCW`.
* `USE_HOME_SENSOR` - This macro can be used to disable the use of the home sensor in systems that don't use it.  Set to `true` if you are using a rotary home sensor.  Set to `false` otherwise.
 
The meaning of most of the rest of the constants should be obvious with the following
exceptions:
* `STEPS_PER_UNIT` - This macro specifies the number of interpolation steps used per unit distance.  For example, if `MAX_SCALE` equals 100 (the normal setting), then setting `STEPS_PER_UNIT` to 1 will cause each
interpolation step to be 1 unit in length.  Setting `STEPS_PER_UNIT` to 2 will cause each interpolation step to be 1/2 unit in length, and so on.<br>
* `HOME_ROT_OFFSET` - I added a rotary axis home sensor and firmware to use it.  The `Home()` function will rotate CW until it detects the home sensor.  Once at the home sensor position the rotary axis will rotate `CCW` for the number of servo motor steps specified by `HOME_ROT_OFFSET`.  This allows for a consistent startup position which can be handy when plotting coordinate array data like `JMCPlot[]`.<br>
* `WIPE_RATIO` and `WIPE_RASTER_INC` - These macros pertain to board clearing (wiping) operations.  The Instructable recommended both 15mm and 8mm ball.  After experimentation, I decided to use a stack of 3mm x 2mm magnets.  Each of these selections has a different footprint when plotting shapes.  Bigger balls create a wider path than smaller balls or disk magnets.  Depending on the marker used, a different width between successive wipe lines can be used.  (There is no need to keep lines very close when using a large ball).  `WIPE_RATIO` is used in `ClearFromIn()` and `ClearFromOut()` to affect a wipe that looks good, but takes the minimum amount of time to execute.  It specifies the ratio value that is passed to `MotorRatios()` to perform the wipe. `WIPE_RASTER_INC` is used by `ClearLeftRight()` for a similar purpose.  It specifies the number of units between successive wipe raster lines.  You may need to experiment with these values to find the best one for your setup.
* `FLOAT_PRECISION` - This specifies the precision that floats generated by `RandomFloat()` will use.  It is specified as 10 to the power of the number of significant digits desired.  For example, setting it to 100.0 will cause 2 significant digits, 1000.0 will cause 3 significant digits, and so on.  A good value seems to be 100.0.

A section of shape related constants specifies limits for many of the shape patterns.  They may be modified as desired.  Their meanings should be obvious.
```
// Shape algorithm limits.
const uint16_t MAX_CYCLES      = 50;        // Maximum cycles to generate when drawing.
const uint16_t MAX_STAR_POINTS = 40;        // Max number of Star() points.
const uint16_t MIN_STAR_POINTS = 3;         // Min number of Star() points.
const float    MAX_STAR_RATIO  = 0.95;      // Max ratio of Star() inside to outside points.
const float    MIN_STAR_RATIO  = 0.1;       // Min ratio of Star() inside to outside points.

const uint16_t MAX_POLY_SIDES  = 8;         // Max number of polygon sides.
const uint16_t MIN_POLY_SIDES  = 3;         // Min number of polygon sides.
const uint16_t MIN_POLY_SIZE   = MAX_SCALE_I / 4;
                                            // Min size of a polygon or star.

const float    MAX_MOTOR_RATIO = WIPE_RATIO;// Max motor ratio value.
const float    MIN_MOTOR_RATIO = 1.0 / MAX_MOTOR_RATIO;
                                            // Min motor ratio value.

const uint16_t MIN_CIRCLE_SIZE = MAX_SCALE_I / 4;
                                            // Min circle size.
const uint16_t MAX_CIRCLE_SIZE = MAX_SCALE_I;
                                            // Max non-lobed circle size.
const uint16_t MIN_CIRCLE_LOBES= 1;         // Min circle lobes.
const uint16_t MAX_CIRCLE_LOBES= 10;        // Max circle lobes.

const uint16_t MIN_SPIRO_FIXEDR= MAX_SCALE_I / 3;
                                            // Minimum spirograph fixed R size.
const uint16_t MIN_SPIRO_SMALLR= 8;         // Minimum spirograph moving circle radius.
const uint32_t SPIRO_NUM_POINTS= 300;       // Number of points to step for
                                            // spirograph shapes.
const float    SPIRO_ANGLE_BASE= PI_X_2 / (float)SPIRO_NUM_POINTS;
                                            // Base angle for spirograph shapes.

const uint16_t MIN_ROSE_VAL    = 1;         // Minimum rose shape num and denom values.
const uint16_t MAX_ROSE_VAL    = 20;        // Maximum rose shape num and denom values.
const uint16_t MIN_ROSE_SIZE   = MAX_SCALE_I / 4;
                                            // Minimum rose x/y size.
const uint16_t MIN_ROSE_RES    = 2;         // Minimum smoothness resolution.
const uint16_t MAX_ROSE_RES    = 64;        // Maximum smoothness resolution.

const uint16_t MIN_CLOVER_VAL  = 1;         // Minimum clover shape radius values.
const uint16_t MAX_CLOVER_VAL  = 10;        // Maximum clover shape radius values.
const uint16_t MIN_CLOVER_SIZE = MAX_SCALE_I / 4;
                                            // Minimum clover x/y size.
const uint16_t MIN_CLOVER_RES  = 6;         // Minimum clover resolution values.
const uint16_t MAX_CLOVER_RES  = 180;       // Minimum clover resolution values.

const uint16_t MIN_ELLIPSE_SIZE = MAX_SCALE_I / 4;
                                            // Minimum ellipse x-axis size;
const float    MIN_ELLIPSE_RATIO = 1.3;     // Minimum ellipse ratio.
const float    MAX_ELLIPSE_RATIO = 8.0;     // Mzximum ellipse ratio.

const uint16_t MIN_SERIES_STEPS = 1;        // Minimum number of series steps.
const uint16_t MAX_SERIES_STEPS = 12;       // Maximum number of series steps.
const uint16_t MIN_SERIES_INC   = 8;        // Maximum size increment for series.
const float    MAX_SERIES_ANGLE = 20.0;     // Maximum angle increment for series.

const uint16_t MIN_HEART_SIZE   = MAX_SCALE_I / 4; // Minimum heart size.
const uint16_t MIN_HEART_RES    = 8;        // Minimum heart size.
const uint16_t MAX_HEART_RES    = 128;      // Minimum heart size.
```

## Adding New Shapes
New shapes are generally implemented as 2 or 3 functions:
* The shape function is used to generate a shape given a specific set of arguments.  A good example is the `Polygon()` function:
```
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
void Polygon(uint16_t numSides, uint16_t size, float rotation)
{
    // Make sure all arguments are within valid limits.
    numSides = constrain(numSides, MIN_POLY_SIDES, MAX_POLY_SIDES);
    float scale = (float)constrain(size, MIN_POLY_SIZE, MAX_SCALE_I);

    // Loop to create the (possibly rotated) polygon.
    for (uint16_t i = 0; i <= numSides; i++)
    {
        float angle = rotation + (PI_X_2 * (float)i) / (float)numSides;
        GotoXY(scale * cosf(angle), scale * sinf(angle));
    }
} // End Polygon().
```
* An (optional) random function is used to generate some valid random arguments for the shape and call the shape with the generated random arguments.  Most shapes have associated random functions.  For example the `RandomPolygon()` function (which is no longer used):
```
/////////////////////////////////////////////////////////////////////////////////
// RandomPolygon()
//
// Calls Polygon() with some random values.
/////////////////////////////////////////////////////////////////////////////////
void RandomPolygon()
{
    // Generate some legal arguments for the call to Polygon().
    uint16_t sides = random(MIN_POLY_SIDES, MAX_POLY_SIDES + 1);
    uint16_t size   = random(MIN_POLY_SIZE, MAX_SCALE_I + 1);
    float    rot    = RandomFloat(0.0, PI_X_2);

    // Show our call.
    LOG_F(LOG_INFO, "Polygon(%d,%d,", sides, size);
    LOG_U(LOG_INFO, RtoD(rot));
    LOG_U(LOG_INFO, ")\n");

    // Make the call to Polygon().
    Polygon(sides, size, rot);
} // End RandomPolygon().
```

* An (optional) series function generates a series of the shape, starting with a random set of arguments, then  increasing the size and angle on successive calls.  The series is generally used with simple shapes that don't repeat for multiple cycles.  For example, `Heart()`, `Polygon()`, `Star()`...  Here is an example:
```
/////////////////////////////////////////////////////////////////////////////////
// PolygonSeries()
//
// Create a series of polygons vith varying size and rotation.
/////////////////////////////////////////////////////////////////////////////////
void PolygonSeries()
{
    LOG_U(LOG_INFO, "PolygonSeries()\n");

    // Generate some legal arguments for the calls to Polygon().
    uint16_t sides = random(MIN_POLY_SIDES, MAX_POLY_SIDES + 1);
    uint16_t size  = random(MIN_POLY_SIZE, (3 * MAX_SCALE_I / 4) + 1);
    float    rot   = RadAngle;

    // Determine how many steps to take, and how much to increase size and angle.
    uint16_t steps = 0;
    uint16_t sizeInc = 0;
    float    rotInc = 0.0;
    GenerateSeriesSteps(size, steps, sizeInc, rotInc);

    // Loop to create the polygon series.
    for (uint16_t i = 0; i < steps; i++)
    {
        LOG_F(LOG_INFO, "Polygon(%d,%d,", sides, size);
        LOG_U(LOG_INFO, RtoD(rot));
        LOG_U(LOG_INFO, ")\n");

        // Make the call to Polygon() and increment our size and rotation.
        Polygon(sides, size, rot);
        size += sizeInc;
        rot += rotInc;
    }
} // End PolygonSeries().
```

To add a new shape, do the following:
*  Create the shape function (`Shape()`).
* For simple shapes, like Star or Polygon, a series function is often a good choice.  For complex shapes, like Spirograph, a randomization function is a good choice.  For shapes that are different based on single or series use, like Circle, both functions may be needed.
* Optionally create a randomization function for the shape (`RandomShape()`).  Generally, a shape will use at least one randomization function or series function.  
* Optiionally create a series function for the shape (ShapeSeries()).
* Add an entry to the `RandomShapes[]` array for the randomization and/or series function.
```
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
    ShapeInfo(RandomWipe, 30),
    ShapeInfo(RandomLines, 50)
}; // End RandomShapes[].
```
* `RandomShapes[]` entries consist of a pointer to the associated function, and a delay value.  The delay value specifies the number of execution cycles that must elapse between executions of the shape.  That is, some shapes should not be generated very often.  This value keeps the shape from occurring too frequently.  For example, wipes should be done rarely, so the delay value used for `RandomWipe()` is 30.  This means that at least 30 other shapes must be displayed before another wipe occurs.

## Random Seed 
At the end of `setup()`, a seed for the random number generator is fed to `randomSeed()`.  A new function was added - `GenerateRandomSeed()` - that generates better random seed values using an unused analog input pin (A2).  This helps to insure that each power-up will generate unique patterns.  The random seed value is logged to the serial port so that it may be saved if needed.  A serial port interface has been added which  allows re-seeding the random number generator to any value.  This provides the ability to manually set the random number generator to a previously reported value in order to duplicate a particular run pattern.
```
    // Seed the random number generator.
    RandomSeed = GenerateRandomSeed();
    randomSeed(RandomSeed);
```

## First Run
The first execution of `loop()` is setup to wipe the board, then display my initials.  This should be taken as an example of what is possible to accomplish at startup.  Feel free to change or remove this code as needed.                
```
    // On first iteration after boot, we wipe the board and display my initials.
    static bool firstTime = true;
    if (firstTime)
    {
        // Wipe the board.
        ClearFromIn();

        // Change this as desired.  It is the power-up greeting.
        RotateToAngle(atan2f((float)JMCPlot[0].y, (float)JMCPlot[0].x));
        PlotShapeArray(JMCPlot, sizeof(JMCPlot) / sizeof(JMCPlot[0]), false);

        // Delay a while to let the user view your awesome work!
        delay(5000);
        firstTime = false;
    }
```

## Remote Command Interface
Limited capability to control the sand table remotely via the serial port was added.  The following commands are accepted (see HandleRemoteCommands() for implementation):
* 'F' (FASTER)      Increase the speed.
* 'S' (SLOWER)      Decreases the speed.
* 'Q'               Restores speed control to the local speed pot.
* 'B' (BRIGHTER)    Increases the LED brightness.
* 'D' (DARKER)      Decreases the LED brightness.
* 'L'               Restores LED control to the local LED pot.
* 'P' (PAUSE)       Pauses motion.
* 'U' (UNPAUSE)     Unpauses motion.
* 'R 'newSeed'
    (RANDOM SEED) Re-seeds the random number generator with the value of
                  'newSeed', homes the axes and clears the board.
* 'G' (GET)         Get the last random seed.
* 'N' (NEXT)        Aborts the current shape and starts the next one.
* 'K' (KEEP ALIVE)  Kicks the watchdog.  If no messages are received from the 
                  serial port after REMOTE_TIMEOUT_MS milliseconds, then all
                  remote settings get cleared and local control is restored.
 
# Conclusions
This has been a great project.  I am very happy with the results.  It gave me a chance to learn more about laser cutting and some useful tools for it.  I would recommend it to anyone with moderate electronics skills.  However, there are a few things I would consider doing differently if I were to make another sand table.
* The Arduino UNO is somewhat under powered for this project.  It is relatively slow by today's standards, and is very memory limited.  For example, MySandTable.ino uses 90% of the Arduino's memory, and runs a little choppy on some of the more complex shapes.  There are two ways to approach this problem.  First, one could offload shape generation code to another processor which could communicate with the Arduino via its USB port.  The Arduino would then only contain communication and motor driver code.  This is the path taken by [another sand table design I've seen](https://github.com/DIY-Machines/Kinetic-Sand-Art-Table).  This is not a bad approach, but it is more expensive, and requires maintaining 2 separate code bases.  I prefer a second path which would be to use a better processor.  The [Adafruit Metro 2350](https://www.adafruit.com/product/6003) is a (mostly) drop in replacement for the Arduino UNO board, but it uses the Raspberry Pi 2350 processor which is almost 10 x as fast and has many times more memory.  This is a great replacement for the UNO, but requires a few firmware changes.  I have completed this update, and will document it in a future project.
* I might consider making the table a bit bigger.  The size of this table is a bit small, and the motors can certainly handle larger parts.  The limit for me would be the size of work pieces that the laser cutter is capable of.
* I would consider painting the bottom of the table black in order to make the sand shapes stand out more.  (I'm not sure if this is a good idea or not, but I'd try it anyway).
* I would update the laser files to add holes that I manually drilled.
* I would relocate the power box to have easier access to the power switch.
* I would consider adding a few buttons to things like abort the current shape, pause, and reboot.  Of course the software would need to change to accommodate these buttons.
* I am considering starting a new project that would automatically take pictures of the table after each shape completes.  This project would use a [Raspberry Pi Zero 2W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/)  or [ESP32 CAM](https://www.instructables.com/Getting-Started-With-ESP32CAM-Streaming-Video-Usi/) which would communicate with the Arduino via USB and would log the startup random seed, the shape being displayed and all of its arguments as well as a picture of the resulting board.  It would be capable of being accessed via a web page.  Stay tuned for more.