https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html - for more reading.
Subsystems are where you define the motors and the actions that a robot can take in the most basic terms.
Essentially, a subsystem as a whole is a grouping of motors and actions that go together (e.g. The swerve drive is a subsystem on its own
comprised of its own swerve modules, just as the "claw" subsystem outlines the motor and actions that the claw intake can do.)

Methods in a subsystem define the most basic actions motors within the subsystem can take, as well as return information from sensors
that are part of the subsystem. To create more complex actions, these are most usually done in the commands section. 
Commands will use the methods you define in subsystems to create complex actions that a robot will carry out.

Before the constructor of each subsystem, you will instantiate each motor that will relate into the subsystem you are creating, as well as
sensors that relate to these motors. Every motor must be defined in a subsystem for it to function. These will always be "private final"
type objects, as they will not be accessed outside the subsystem. This way, it ensures best practice in that you will not access a motor
directly from a command - the action should always pass through a subsystem method.

In the constructor of the subsystem class, you will define each of these motors with a motor controller object. In just about all cases for a motor
and sometimes for a sensor, you will create a new object of that motor controller from the library that the motor is provided from.
motorIWant = new TalonSRX(parameters); // e.g 
Sometimes, for a sensor that will come directly from a motor, you will call a method to reference an object already created as a result
of the motor instance creation. For example, a SparkMAX encoder is a RelativeEncoder class object that is obtained from calling getEncoder
on a SparkMAX controller.
The motors will require a CAN Bus ID as a parameter to know where to point the instructions you've coded. These should be outlined in Constants.java
as best practice, but you can see within this code that I didn't follow that and sometimes defined IDs just in subsystem. It's not
the best practice to do so because it is much easier to find a value in Constants.java and change it to change every reference rather than find 
each reference and change each of them. SparkMAX controllers also require a enum containing whether it is brushless or not, but for the use cases
of SparkMAX controllers (NEOs) you will most likely select MotorType.kBrushless. Motor model does not matter past that in code.

After that, you will define methods that will make up the robot functions. You can look at the methods within the current code for
examples as to how to make your methods. Each library has different functions that you'll have to access Java documentation for. 
An important tip - hovering over text will tell you all the info VSC has on whatever you're hovering on. This is useful for seeing which
parameters a library function wants and satisfying those requirements. Make functions basic and easy to navigate - you'll have to comprise
more complex functions with these basic building blocks. 

A Javadoc coment is used as a super easy reference tool that VSC automatically references when you hover over your own methods.
It is good practice to use one of these for every method that takes in parameters - you can explain each one there and it will
automatically pop up when you hover over the method anywhere else. (use these a lot)

The periodic() of a subsystem, lastly, is executed every scheduler tick (20ms) as long as the robot runs. Useful for printing info to 
SmartDashboard if you need to see some sensor values. 
