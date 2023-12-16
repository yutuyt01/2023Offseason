// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final DoubleSolenoid clawPneumatic;

  private final CANSparkMax clawMotor;
  private final SparkMaxPIDController pid;

  private final RelativeEncoder clawCoder;
  private final DigitalInput clawLimitSwitch;

  public ClawSubsystem() {
    clawMotor = new CANSparkMax(39, MotorType.kBrushless);
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    clawMotor.setIdleMode(IdleMode.kBrake);

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    clawCoder = clawMotor.getEncoder();
    // A PID controller is a Proportion, Integral, and Derivative controller. It outputs numbers based on the input given, the parameters of the PID, and the "setpoint" or target value.
    pid = clawMotor.getPIDController();
    pid.setP(1);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug in limit switches into here normally. Essentially, this declaration points to the number 9 slot on the DIO. 
    clawLimitSwitch = new DigitalInput(9);
    // This doesn't exist on the robot, but I didn't remove it from when we did. A solenoid is a pnuematic controller that controls the off and on of the pnuematic. You'll define the forward and close ports on the pnumeatics hub for this. If it's reversed, swap the numbers.
    clawPneumatic = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
    // This ensured it was closed at the start of a match.
    clawPneumatic.set(Value.kReverse);
  }
  /**
   * Simple function to spin the claw motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void spinClaw(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    clawMotor.set(speed);
  }
  /**
   * Simple function to stop the claw. It is good to have this as opposed to running spinClaw(0), because it ensures there is no error.
   */
  public void stopClaw() {
    clawMotor.set(0);
  }
  
  /**
   * Returns the status of the solenoid. Notice that it is not a boolean, but a Value enum.
   * @return Value type object that defines the pnuematic position. (Value.kReverse vs Value.kForward vs Value.kOff)
   */
  public Value isClawOpen() {
    return clawPneumatic.get();
  }

  /**
   * Toggles the status of the claw. toggle() goes from kReverse to kForward and vice versa - no result if used on kOff.
   */
  public void toggleClaw() {
    clawPneumatic.toggle();
  }

  /**
   * Sets the position of the claw open.
   */
  public void openClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Sets the position of the claw closed, or reverse.
   */
  public void closeClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Returns the claw encoder position. The spinning wheels do have a position - the encoder is located inside the NEO.
   * @return The double encoder position.
   */
  public double getClawEncoder() {
    return clawCoder.getPosition();
  }

  /**
   * Relaxes the claw by setting there to be no input.
   */
  public void releaseClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kOff);
  }

  /**
    * Implementation of a SparkMAX's internal PID controller. This feature is specific to SparkMAXes.
    * This method sets the reference (or more accurately, setpoint) for the PID to approach. When called, it sets the reference point to the current position)
    * This is done in the intention to cause it to keep still - as it is trying to approach the point it was called at)
   */
  public void holdClawPosition() {
    pid.setReference(getClawEncoder(), ControlType.kPosition);
  }

  /**
   * Returns a boolean on whether the limit switches were tripped in the intake mechanism. Notice how the value is negated - this is a simple code change due to wiring necessities.
   * It is much easier to change code like this than wires.
   * @return Boolean for whether limit is tripped. True is tripped, false is not.
   */
  public boolean isClawLimitSwitchTripped() {
    return !clawLimitSwitch.get();
  }
  
  /**
   * Periodic function standard to all subsystems.
   */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw Limit Switch", isClawLimitSwitchTripped());
    SmartDashboard.putNumber("ClawCoder", getClawEncoder());
  }
}
