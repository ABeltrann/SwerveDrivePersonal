// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class CalibrateGyroCmd extends CommandBase {
  /** Creates a new CalibrateGyroCmd. */
  private SwerveSubsystem swerveSubsystem;
  public CalibrateGyroCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.calibrateGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
 public boolean runsWhenDisabled(){
   return true;
 }
  @Override
  public boolean isFinished() {
    return true;
  }
}
