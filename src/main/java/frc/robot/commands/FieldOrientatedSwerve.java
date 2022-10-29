// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FieldOrientatedSwerve extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xValueSupplier;
  private final DoubleSupplier yValueSupplier;
  private final DoubleSupplier rotValueSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldOrientatedSwerve(DoubleSupplier xValueSupplier, DoubleSupplier yValueSupplier, DoubleSupplier rotValueSuppllier, SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.xValueSupplier = xValueSupplier;
    this.yValueSupplier = yValueSupplier;
    this.rotValueSupplier = rotValueSuppllier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds fieldChassisSpeeds;
    double xValue = xValueSupplier.getAsDouble();
    double yValue = yValueSupplier.getAsDouble();
    double rotValue = rotValueSupplier.getAsDouble();

     fieldChassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(xValue, yValue, rotValue, swerveSubsystem.getHeading());
    SwerveModuleState[] states = SwerveSubsystemConstants.kDriveKinematics.toSwerveModuleStates(fieldChassisSpeeds);
    swerveSubsystem.setSwerveModuleStates(states );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
