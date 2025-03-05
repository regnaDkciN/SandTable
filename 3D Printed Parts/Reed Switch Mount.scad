ReedLen = 16;
ReedWid = 2;
ReedAndWireLen = ReedLen + 10;
ReedAndWireWid = 10;
NubDia = 3;
Margin = 1.4;
BoxWid = 2 * ReedWid + 2 * Margin + NubDia;
BoxLen = ReedAndWireLen + Margin;
BoxH = ReedWid + 2 * Margin;
WireWid = 1.5;
MountHoleD = 3.1;         // Diameter of side mount holes.

echo(BoxWid);

module Box()
{
    difference()
    {
        // Basic box.
        cube([BoxLen, BoxWid, BoxH], center = true);

        //  Cut out space for the reed soitch and one of its leads.
        translate([-Margin, (NubDia + ReedWid) / 2, Margin]) 
            cube([BoxLen, ReedWid, BoxH], center = true);
        translate([-Margin, -(NubDia + ReedWid) / 2, Margin])
            cube([BoxLen, ReedWid, BoxH], center = true);
            
        // Add space for the rounded off the upper wire guide.  
        translate([(BoxLen - (ReedWid + NubDia / 2)) / 2 - Margin, 0, Margin]) 
            cube([ReedWid + NubDia / 2, NubDia + .01, BoxH], center = true);
    }

    // Round off the upper wire guide.
    translate([(BoxLen - NubDia) / 2 - Margin - ReedWid, 0, -BoxH / 2]) 
        cylinder(d = NubDia, h = BoxH, $fn = 30);
}

module Tab()
{
    difference()
    {
        // Create the tab circlw.
        cylinder(d = BoxWid, h = Margin, $fn = 30);
        
        // Cut off half of the tab to make a semicircle.
        translate([0, -BoxWid, -.01])
            cube([BoxWid * 2, BoxWid * 2, BoxH]);
            
        // Cut out the mounting hole.
        translate([-2, 0, -.01])
            cylinder(d = MountHoleD, h = BoxH, $fn = 30);
    }
    
    // Extend the tab to allow room for the mounting screw head.
    translate([0, -BoxWid / 2, 0]) 
        cube([MountHoleD / 2, BoxWid, Margin]);
}

module ReedSwitchMount()
{
    // Start with the basic box.
    Box();
    
    // Add the mounting tabs.
    translate([-(BoxLen - BoxWid) / 2, (BoxWid + MountHoleD) / 2, BoxH / 2 - Margin])
        rotate([0, 0, -90]) Tab();
    translate([(BoxLen + MountHoleD)/ 2, 0, BoxH / 2 - Margin])
        rotate([0, 0, 180]) Tab();
}

ReedSwitchMount();
