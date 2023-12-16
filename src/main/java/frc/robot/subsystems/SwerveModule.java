package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    /** Swerve module class that encompasses one full swerve module and the motors/encoders that go with it. 
     * Think of this as the template that SwerveSubsystem uses to make things simpler. It would be pretty messy to declare
     * all 4 modules (8 motors) in the same place when you need control over each pair of two individually for swerve.
     * Instead, functions that are needed for each module are placed here, and more general ones that apply to all in subsystem.
     * Definitely check out README-SwerveClasses before going through this.
    */
    private final CANSparkMax driveMotor; 
    private final TalonSRX turningMotor;

    // This is the object for NEO encoders.
    private final RelativeEncoder driveEncoder;
    //private final Encoder turningEncoder;

    /** See PIDLoops.txt for info on PID loops and how to tune them. This creates an object which we'll declare values for in the
     * constructor below. Then, we can call .calculate(input, setpoint) on this object which returns the output (most likely routed to your
     * motor movement function) to make it do work.
    */
    private final PIDController turningPidController;

    /** Absolute encoders can go to an AnalogInput instead of a DigitalInput. 
     * These output a voltage between 0-5V based on the rotation of the wheel. 
     * However, due to the nature of absolute encoders, they will always be the same even after you turn the robot off - no way to
     * set it to be zero at a certain point. An offset may be needed to tell the robot how far the number should be from zero to be 
     * "zeroed".
     * There are some things done here that I would not repeat.
     * 
     * This code uses a literal "magic number" that was achived by spinning the wheel 360 5 times and seeing the encoder counts move
     * to calculate a conversion factor for pulses to radians. Reason being that the gear ratio just made no sense in comparison to
     * the numbers we were getting. Hopefully not a problem with the mk4i's.
     * 
     * There is no autohoming implementation that works in this code written here. I'ts been a while so I don't exactly remember why the
     * implmentation in the Zero to Auto video doesn't work. Again hopefully not a problem with mk4i's.
    */
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    // See comment above for this value.
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int[] angleEncoderIds, boolean isFL) {
        // The this keyword allows for less confusion when passing params into variables. You can name them the same thing, but only
        // when assigning like this.variable = variable.
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new TalonSRX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setNeutralMode(NeutralMode.Coast);
        /** Specifically tells the Talon that the input device is a quadrature encoder.
         * A quadrature encoder is used for many applications in FRC, and is pretty much the only use besides absolute.
         * They do not track absolute position, but can provide direction of rotation information.
         * This encoder can then be accessed through getSelectedSensorPosition() and other related methods.
         * */  
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Assigns the object inherent to the NEO to the RelativeEncoder.
        driveEncoder = driveMotor.getEncoder();
        //turningEncoder = new Encoder(angleEncoderIds[0], angleEncoderIds[1]);
        
        /** For on the fly swerve measurements, it is simpler if the code itself can calculate position/speed/angular velocity.
         * These tell the encoders how far a pulse/revolution is compared to a meter. Right click the values and go to
         * definition to see how they are calculated.
         * For a RelativeEncoder object, setPositionConversionFactor/setVelocityConversionFactor converts 
         * a revolution of the motor to distance using your number. The encoder has more resolution than that,
         * but it makes it easier to convert because you don't need to know the resolution of the NEO encoder.
         * 
         * Then, you can access these adjusted values using getPosition()/getVelocity() to make life easier.
         * 
         * Below, the conversions for the turning encoder are commented out and applied directly in the functions that 
         * return values for position/velocity. I'm actually not sure why that is - I think it's because I was troubleshooting
         * errors in the conversion and was seeing if that was the problem (it was not). 
         * That magic number is in kTurningEncoderPPRad. There's no math that makes that number make sense, but it works.
         */
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderPPRad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        /** I used a different PID controller for the front left motor because it was harder to move. 
         * However, we then fixed the problem with that module, so essentially this distinction has no use - just was never removed.
         * (Both have proprotions of 1.5.)
         * Assigns PIDControllers for each swerve module. 
         */
        if (isFL = false) {
            turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0);
        } else {
            turningPidController = new PIDController(ModuleConstants.kPTurningFL, 0.001, 0);
        }

        // PIDLoops.txt
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Running a function to resetencoders to make sure all counts are zero on creation. Redundant and not necessary. 
        resetEncoders();
    }

    /**
     * Returns converted drive position.
     * @return Distance that the motor has moved converted to meters travelled on the ground.
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns converted angle motor position.
     * @return Location of the wheel relative to zero in radians.
     */
    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderPPRad * 0.25;
    }

    /**
     * Returns raw angle encoder counts.
     * @return Turning motor position in encoder pulses.
     */
    public double getRawTurningEncoder() {
        return turningMotor.getSelectedSensorPosition();
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * @return Velocity of the drive wheel in m/s.
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the angular velocity of the wheel in rad/s.
     * @return Angular velocity of the wheel in rad/s.
     */
    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity() * (10) * ModuleConstants.kTurningEncoderPPRad * 0.25;
    }

    /**
     * Returns the absolute position of the wheel, including offsets relative to straight forward.
     * @return Radians deviated from straight forward.
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the raw voltage from the absolute encoder on this swerve module.
     * @return Voltage from 0-5V.
     */
    public double getRawAbsoluteEncoderVoltage() {
        return absoluteEncoder.getVoltage();
    }
    
    /**
     * Returns the PIDController object used turning this module.
     * I'm not sure why this is implemented. The PIDController is marked final and can't be edited and I'm pretty sure I didn't use this
     * anywhere else.
     * @return PIDController object used to control movement of the turning motor.
     */
    public PIDController getPIDController() {
        return turningPidController;
    }

    /**
     * Resets drive encoder for the module. Now that I think about it I didn't really try to see if this would work again.
     * As a first exercise, uncomment the turningencoder line and see if autohoming works :P
     * 
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turningEncoder.setPosition(/*getAbsoluteEncoderRad()*/);
    }

    /**
     * This will set the encoder positions to based apon an angle input
     * @param angle This is expecting an angle in radians -2pi - 2pi
     */
    // Another byproduct of the attempt on autohoming
    public void setEncoderAngle(double angle){
        angle = angle * (1 / (ModuleConstants.kTurningEncoderPPRad * 0.25));
        turningMotor.setSelectedSensorPosition(angle);
    }

    /**
         * Returns the SwerveModuleState object for this swerve module.
         * SwerveModuleState is a snapshot of the drive velocity and rotation of this swerve module)
         * It's used in calculating the correct module states that are to be output to the wheels.
         * Say for example, this swerve module is moving at 2m/s and is straight forward.
         * However, based on code from the joystick command, it's requesting 5m/s and left.
         * This object will allow for two PID controllers to be used in a single function to output the correct
         * drive and turning power percentages to reach the requested state.
         * (See the SwerveJoystickCmd command.)
     * @return SwerveModuleState object of current drive status.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Similar to above, but contains a drive position as opposed to velocity. Useful for autonomous driving.
     * @return SwerveModulePosition object of current drive status.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }
    /**
     * An attempt to test something by setting the motor to directly spin 
     */
    public void testTurnMotor() {
        turningMotor.set(ControlMode.PercentOutput, 0.5);
    }
    /**
     * Writes desired swerve module state to the drive and turning motors.
     * @param state Desired swerve module state. 
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        /* Optimization stops a wheel from rotating too far if an equivalent angle that is closer to the target exists.
         * It edits the state before any calculations after this.
        */
        state = SwerveModuleState.optimize(state, getState().angle);
        // As the speed approaches the physical max, the motor will max out at 1.
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        double out =  turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(ControlMode.PercentOutput, out);
        // Debug
        SmartDashboard.putNumber("PID turn output" + absoluteEncoder.getChannel(), out);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    /**
     * A stop function is always good in any code you write for robot subsystems.
     * Stops module movement.
     */
    public void stop() {
        driveMotor.stopMotor();
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
