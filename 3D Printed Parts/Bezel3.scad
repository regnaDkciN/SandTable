BaseHeight = 3;
RailLipHeight = 4;
RailLipWidth = 2;
RailSlotWidth = 3.2;
RailWidth = RailSlotWidth + RailLipWidth * 2;
RailHeight = BaseHeight + RailLipHeight;
BaseDepth = RailWidth + 15;
WingAngle = 135;
LeftRailLength = 45;
RightRailLength = 34;
CentralRailLength = 63;
fudge = 0.02;

module Rail(length)
{
    cube([length, BaseDepth, BaseHeight]);
    difference()
    {
        cube([length, RailWidth, RailHeight]);
        translate([-.5, RailLipWidth, BaseHeight])
            cube([length + 1, RailSlotWidth, RailHeight]);
        translate([-fudge / 2, RailLipWidth + RailSlotWidth - fudge, BaseHeight])
            cube([RailLipWidth + fudge, RailLipWidth + fudge, RailHeight]);
        translate([length - RailLipWidth - fudge / 2, 
                    RailLipWidth + RailSlotWidth - fudge / 2, 
                    BaseHeight])
            cube([RailLipWidth + fudge, RailLipWidth + fudge, RailHeight]);
    }
}

module Rails()
{
    Rail(CentralRailLength);
    
    rotate([0, 0, WingAngle + 180]) 
        translate([-RightRailLength, 0, 0]) Rail(RightRailLength);    
    
    translate([CentralRailLength, 0, 0]) rotate([0, 0, 180 - WingAngle])
        Rail(LeftRailLength);   
}

//Base(60);
Rails();


