module OrigGear()
{
    translate([-10, -10, 0])
    linear_extrude(height = 6.4, center = true, convexity = 10)
		import(file = "Gear Hole Test 1.10.dxf", convexity = 10, $fn = 120);
}


module ScaledGear(s)
{
    intersection()
    {
        #OrigGear();
        scale([s, s, 1]) OrigGear();
    }
}

ScaledGear(.98);



