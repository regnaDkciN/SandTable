// OpenSCAD script to modify the original Arduino Bumper as follows:
//   - Enlarge in the X and Y axes to fit a slightly larger board.
//   - Shrink in the Z direction based on the current project need.
//   - Shrink the diameter of the mounting holes so that M3 screws will thread
//     directly into them rather than passing through.
//   - Add notch for micro SD card.
//   - Add notch for power switch.


XScale           = 1.01;        // X axis scale for entire assembly.
YScale           = XScale;      // Y axis scale for entire assembly.
ZScale           = 1.0;         // Z axis scale for entire assembly.
SmallHoleD       = 2.8;         // Diameter of (shrunken) mounting holes.
OrigThickness    = 2.6;         // Thickness of original assembly.
DesiredThickness = 2.0;         // Final thickness of the assembly.
MountSize        = 15.0;        // Length of mount cube sides.
MountThickness   = 3.2;         // Thickness of side mounts.
MountHoleD       = 3.1;         // Diameter of side mount holes.
HoleThickness    = DesiredThickness - 0.8;
                            // Thickness of new (shrunken) mounting holes.
AlignHoles    = false;      // Set true to add center alignment pins.
                            // Be sure to set to false before final render.
SdNotch            = [10, 14, 2]; // Size of cube used to cut out SD card notch.
PwrNotch           = [7, 10, 2]; // Size of cube used to cut out power notch.


MountingHoleCoords =        // X/Y coordinates of the 4 mounting holes.
            [[31.74, 8.70], [31.55, -19.05], [-19.05, 24.12], [-20.30, -24.12]];


module AlignmentPin()
{
    cylinder(d = 0.1, h = 5.0, $fn = 30);
}

// Import the original model, and display mounting hole alignment centers if requested.
module Original()
{
    import("Arduino_Bumper_0006.stl", convexity = 10);
    // Display the mountinng hole centers for alignment purposes if requested.
    if (AlignHoles)
    {
        for (c = MountingHoleCoords)
        {
            translate([c.x, c.y, 0.0]) AlignmentPin();
        }
    }
}


// Scale the original assembly.
module ScaledOriginal(s = [1.0, 1.0, 1.0])
{
    difference()
    {
        translate([0.0, 0.0, -(OrigThickness - DesiredThickness)]) scale(s) Original();
        translate([0.0, 0.0, -5]) cube([100.0, 100.0, 10.0], center = true);
    }
}


// A cube containing the (shrunken) mounting hole.  Adds alignment centers if desired.
module Hole()
{
    // Create a small cube with a shrunken mounting hole.
    difference()
    {
        translate([0.0, 0.0, HoleThickness / 2 + .4]) 
            cube([4.0, 4.0, HoleThickness], center = true);
        translate([0.0, 0.0, -1.1]) cylinder(d = SmallHoleD, h = 4.0, $fn = 30);
    }
    // Display the mountinng hole centers for alignment purposes if requested.
    if (AlignHoles)
    {
        translate([0.0, 0.0, 1.0]) AlignmentPin();
    }
}


// A side mount containing a mounting hole.
module Mount()
{
    difference()
    {
        // The mounting cube.
        cube([MountSize, MountSize, MountThickness]);
        // The mounting hole.
        translate([MountSize / 2, MountSize / 2, -1])
            cylinder(d = MountHoleD, h = MountThickness + 2.0, $fn = 30);
    }
}


// Final assembly with possibly enlarged X and Y axes and shrunken Z axis and mounting holes.
module FinalAssembly(s = [1.0, 1.0, 1.0])
{
    difference()
    {
        union()
        {
            ScaledOriginal(s);
            for (c = MountingHoleCoords)
            {
                translate([c.x * XScale, c.y * YScale, 0.0]) Hole();
            }
            
            // Add side mounts.
            translate([-20.0, -27.5 - MountSize, 0.0]) Mount();
            translate([-20.0,  27.5, 0.0]) Mount();
        }
        translate([27, -17, 2.8]) cube(SdNotch);
        translate([-16, -32, 3.2]) cube(PwrNotch);
    }
}

FinalAssembly([XScale, YScale, ZScale]);
