// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  /** Controls everything relating to the Johnson motor extension system. */
  private final TalonSRX extendMotor;
  // A DigitalInput is pretty much anything that goes into the DIO slots (numbered 0-9) on the RIO. This can be encoders/limit switches, etc.
  private final DigitalInput extensionLimitSwitch;
  public ExtensionSubsystem() {
    extendMotor = new TalonSRX(21);
    // Neutral mode is what you want the motor's action to default to. (Brake, Coast)
    extendMotor.setNeutralMode(NeutralMode.Brake);
    // Digital input slot 8 for the limit switch.
    extensionLimitSwitch = new DigitalInput(8);
  }

  /**
   * Extend arm to passed encoder position. 
   * @param movePosition Encoder count location.
   */
  public void extendArmPosition (double movePosition) {
    extendMotor.set(ControlMode.Position, movePosition);
  }

  /**
   * Reset extension encoder to position specified. We used an input value here because when zeroing I wanted to give it some space 
   * between hitting the limit switch and the actual zero position. The zero position was no longer right against the limit switch
   *  - it was slightly before so sending to zero doesn't slam the back of the extension.
   * Note how this doesn't call on a specific encoder object. This is because the embedded encoder in the Johnson is
   * hooked directly to the SRX, meaning that the readings are reachable through the SRX object and methods.
   * 
   * @param resetPos Reset position to set value.
   */
  public void resetExtensionEncoder(double resetPos) {
    extendMotor.setSelectedSensorPosition(resetPos);
  }

  /**
   * Returns the encoder count position of the Johnson.
   * @return Encoder count.
   */
  public double getExtensionEncoder() {
    return extendMotor.getSelectedSensorPosition();
  }

  /**
   * Returns the limit switch status in a boolean. Notice the ! operator - negating the value. 
   * If you find that this is giving you a true value when you want a false, it's much easier to negate in code vs swapping wires.
   * @return Boolean value of limit switch press status.
   */
  public boolean isLimitSwitchTripped() {
    return !extensionLimitSwitch.get();
  }

  /**
   * This runs a motor at a certain speed instead of to a position. This was used for moving the motor slowly to home it.
   * @param inputSpeed Speed from -1 to full reverse to 1 for full forward.
   */
  public void extendArmSpeed(double inputSpeed) {
    extendMotor.set(ControlMode.PercentOutput, inputSpeed); 
  }

  /**
   * Periodic function for the subsystem that repeats. Used here to push encoder counts to dashboard constantly.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extenson Motor Position", getExtensionEncoder());
    SmartDashboard.putBoolean("limit switch trip status", isLimitSwitchTripped());
  }
}
