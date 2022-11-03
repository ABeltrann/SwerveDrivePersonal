// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroCalibrationCmd extends CommandBase {
  /** Creates a new GyroCalibrationCmd. */
  SwerveSubsystem swerveSubsystem;
  public GyroCalibrationCmd(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.calibrateGyro();
  }  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

}