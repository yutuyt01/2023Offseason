// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Turn360 extends CommandBase {
  /** Creates a new Turn360. */
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveModuleState input;
  public Turn360(SwerveSubsystem swerveSubsystem, SwerveModuleState input) {
    this.swerveSubsystem = swerveSubsystem;
    this.input = input;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] sms = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), input}; 
    swerveSubsystem.setModuleStates(sms);
    SmartDashboard.putNumber("test", 1);
    SmartDashboard.putString("testBR", input.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
