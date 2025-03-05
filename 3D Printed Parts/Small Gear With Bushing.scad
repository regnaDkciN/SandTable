BushingD = 11;
BushingH = 2.4;

module OrigGear()
{
    translate([-10, -10, 0])
    linear_extrude(height = 3, center = true, convexity = 10)
		import(file = "Gear Hole Test 1.10.dxf", convexity = 10, $fn = 120);
}


module ScaledGear(s)
{
    intersection()
    {
        OrigGear();
        scale([s, s, 1]) OrigGear();
    }
}

module Bushing(s)
{

    intersection()
    {
        ScaledGear(s);
        translate([0, 0, -3]) cylinder(d = BushingD, h = 6, $fn = 60);
    }
}

module GearWithBushing(s)
{
    translate([0, 0, BushingH]) Bushing(s);
    ScaledGear(s);
}



GearWithBushing(.98);