// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveSubsystemConstants;;

public class SwerveSubsystem extends SubsystemBase {
  SwerveDriveOdometry odometry;
  AHRS NavX;
  Pose2d pose2d;
  
 
  public SwerveSubsystem() {
  odometry = new SwerveDriveOdometry(SwerveSubsystemConstants.kDriveKinematics, NavX.getRotation2d());
  NavX  = new AHRS(Constants.SwerveSubsystemConstants.kGyroId);
  pose2d = new Pose2d();
      
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
