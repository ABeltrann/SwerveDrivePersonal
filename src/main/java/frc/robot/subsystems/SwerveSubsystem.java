// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveSubsystemConstants;

public class SwerveSubsystem extends SubsystemBase {
  private  SwerveDriveOdometry odometry;
 private final AHRS NavX;
private  Pose2d pose2d;
 private final SwerveModule frontLeftModule = new SwerveModule(SwerveSubsystemConstants.kFrontLeftTurningID, SwerveSubsystemConstants.kFrontLeftFowardID, 
  SwerveSubsystemConstants.kFrontLeftFowardEncoderID, SwerveSubsystemConstants.kFrontLeftTurningEncoderID, SwerveSubsystemConstants.kFrontLeftFowardInverted , 
  SwerveSubsystemConstants.kFrontLeftTurningInverted, SwerveSubsystemConstants.kFrontLeftAbsoluteEncoderOffset, SwerveSubsystemConstants.kFrontLeftAbsoluteEncoderInverted, 
  SwerveSubsystemConstants.kFrontLeftAbsoluteEncoderID, "Front Left Module");

  private final SwerveModule frontRightModule = new SwerveModule(SwerveSubsystemConstants.kFrontRightTurningID, SwerveSubsystemConstants.kFrontRightFowardID, 
  SwerveSubsystemConstants.kFrontRightFowardEncoderID, SwerveSubsystemConstants.kFrontRightTurningEncoderID, SwerveSubsystemConstants.kFrontRightFowardInverted , 
  SwerveSubsystemConstants.kFrontRightTurningInverted, SwerveSubsystemConstants.kFrontRightAbsoluteEncoderOffset, SwerveSubsystemConstants.kFrontRightAbsoluteEncoderInverted, 
  SwerveSubsystemConstants.kFrontRightAbsoluteEncoderID, "Front Right Module");

  private final SwerveModule BackLeftModule = new SwerveModule(SwerveSubsystemConstants.kBackLeftTurningID, SwerveSubsystemConstants.kBackLeftFowardID, 
  SwerveSubsystemConstants.kBackLeftFowardEncoderID, SwerveSubsystemConstants.kBackLeftTurningEncoderID, SwerveSubsystemConstants.kBackLeftFowardInverted , 
  SwerveSubsystemConstants.kBackLeftTurningInverted, SwerveSubsystemConstants.kBackLeftAbsoluteEncoderOffset, SwerveSubsystemConstants.kBackLeftAbsoluteEncoderInverted, 
  SwerveSubsystemConstants.kBackLeftAbsoluteEncoderID, "Back Left Module");

  private final SwerveModule BackRightModule = new SwerveModule(SwerveSubsystemConstants.kBackRightTurningID, SwerveSubsystemConstants.kBackRightFowardID, 
  SwerveSubsystemConstants.kBackRightFowardEncoderID, SwerveSubsystemConstants.kBackRightTurningEncoderID, SwerveSubsystemConstants.kBackRightFowardInverted , 
  SwerveSubsystemConstants.kBackRightTurningInverted, SwerveSubsystemConstants.kBackRightAbsoluteEncoderOffset, SwerveSubsystemConstants.kBackRightAbsoluteEncoderInverted, 
  SwerveSubsystemConstants.kBackRightAbsoluteEncoderID, "Front Right Module");
 
  public SwerveSubsystem() {
  odometry = new SwerveDriveOdometry(SwerveSubsystemConstants.kDriveKinematics, new Rotation2d(0));
  NavX  = new AHRS(Constants.SwerveSubsystemConstants.kGyroId);
  pose2d = new Pose2d();


  

      
  }

  public Rotation2d getHeading(){
   return Rotation2d.fromDegrees(NavX.getAngle());
  }

  public void resetOdometry( Pose2d pose){
    odometry.resetPosition(pose , getHeading());
  }

  @Override
  public void periodic() {
    pose2d = odometry.update(getHeading(), frontLeftModule.getState(),frontRightModule.getState(),BackLeftModule.getState(), BackRightModule.getState() );

    

  }

  public void setSwerveModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveSubsystemConstants.kAbsoluteMaxSpeeds);
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    frontRightModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[3]);




  }


}
