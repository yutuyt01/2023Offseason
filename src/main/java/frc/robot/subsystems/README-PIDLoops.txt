PID loops (proportion, integral, derivative) are very important for a lot of motion control in FRC. Essentially, by tweaking these three numbers,
you can get smooth motion between your starting point and ending point. This requires a lot of tuning to get just right, but it's cool to see it happen.

Normally, PID output is obtained by making a PIDController object or command and recieving outputs from that. SparkMAXes also have an internal PID controller that can be utilized. 

Proportion (P) is a measure of simply how much power the motor should recieve. This number requires fine tuning to get right, but it is very important.
Less power, decrease this. More power, increase. Change this number FIRST before changing anything else.

Integral (I) is a measure of how much the motor will attempt to correct its mistakes. With a good proprtion but no integral, a motor
will oscillate around its setpoint. This is where integral comes in. Start by adding very miniscule amounts of it until your osciallation
is either removed or dampened. Change this after proportion.

Derivative (D) is a measure of how delayed the controler will attempt to output things. This is also a very miniscule number,
normally smaller than integral. By increasing this, your motion will become smoother but if you increase it too much it will overshoot consistently.

There are some other important parts of a PID controller.
Tolerance is a measure of how much the setpoint can be off from the current value before it quits. The units for this is the same as the measure you are using to control the controller.
Continuous output is telling the PID controller where it can start off and loop around to the same value - super important for measures of angle so that the
code can smoothly transition from one end to the other. 
The setpoint of a PID controller is the point where it is trying to get to. 
If |setpoint - input| < tolerance, then the controller will quit. (Most likely. They won't do this if they overshoot)
