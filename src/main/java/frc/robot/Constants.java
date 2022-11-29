// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;


public final class Constants {
    public static final class  SwerveModuleConstants{
    public static final double kWheelDiameter = Units.inchesToMeters(1.5);
    //To be changed
    public static final double kDriveGearRatio = 1/8.14;
    public static final double kTurnGearRatio = 1/12.8;
    public static final double kDriveEncoder2Meter = kWheelDiameter * Math.PI * kDriveGearRatio;
    public static final double kTurnEncoder2Radians = kTurnGearRatio * Math.PI * 2;
    public static final double kDriveEncoderMetersPerSecond = kDriveEncoder2Meter/60;
    public static final double kTurnEncoder2RadiansPerSecond = kTurnEncoder2Radians/60;
    //Placement values, need to be tuned
    public static final double kPTurning = 0;
    public static final double kITurning = 0;
    public static final double KDTurning = 0;
    public static final double KFfowardspeed = 0;
    }
    public static final class SwerveSubsystemConstants{
    //measurements of the width & length between each wheel
    public static final double kDriveWidth = Units.inchesToMeters(25);
    public static final double kDriveLength = Units.inchesToMeters(25);
    //takes in the position of each module relative to the center of robot, order is Front Left, FrontRight, BackLeft, BackRight 
    public static final SwerveDriveKinematics kDriveKinematics = 
    new SwerveDriveKinematics(new Translation2d(kDriveWidth/2, -kDriveLength/2),
    new Translation2d(kDriveWidth/2, kDriveLength/2),
    new Translation2d(-kDriveWidth/2, -kDriveLength/2),
    new Translation2d(kDriveWidth/2, -kDriveLength/2)
    );
   
    public static final Port kGyroId = null;
    //Module constants, will be finished when swerve arrives
    public static final int kFrontLeftTurningID = 0;
    public static final int kFrontLeftFowardID = 0;
    public static final int kFrontLeftFowardEncoderID = 0;
    public static final int kFrontLeftTurningEncoderID = 0;
    public static final boolean kFrontLeftFowardInverted = false;
    public static final boolean kFrontLeftTurningInverted = false;
    public static final Double kFrontLeftAbsoluteEncoderOffset = null;
    public static final boolean kFrontLeftAbsoluteEncoderInverted = false;
    public static final int kFrontLeftAbsoluteEncoderID = 0;
    
    public static final int kFrontRightTurningID = 0;
    public static final int kFrontRightFowardID = 0;
    public static final int kFrontRightFowardEncoderID = 0;
    public static final int kFrontRightTurningEncoderID = 0;
    public static final boolean kFrontRightFowardInverted = true;
    public static final boolean kFrontRightTurningInverted = false;
    public static final Double kFrontRightAbsoluteEncoderOffset = 13.5;
    public static final boolean kFrontRightAbsoluteEncoderInverted = false;
    public static final int kFrontRightAbsoluteEncoderID = 0;
    
    public static final int kBackLeftTurningID = 0;
    public static final int kBackLeftFowardID = 0;
    public static final int kBackLeftFowardEncoderID = 0;
    public static final int kBackLeftTurningEncoderID = 0;
    public static final boolean kBackLeftAbsoluteEncoderInverted = false;
    public static final boolean kBackLeftTurningInverted = false;
    public static final Double kBackLeftAbsoluteEncoderOffset = null;
    public static final boolean kBackLeftFowardInverted = false;
    public static final int kBackLeftAbsoluteEncoderID = 0;
   
    public static final int kBackRightTurningID = 0;
    public static final int kBackRightFowardID = 0;
    public static final int kBackRightFowardEncoderID = 0;
    public static final int kBackRightTurningEncoderID = 0;
    public static final boolean kBackRightFowardInverted = true;
    public static final boolean kBackRightTurningInverted = false;
    public static final Double kBackRightAbsoluteEncoderOffset = null;
    public static final boolean kBackRightAbsoluteEncoderInverted = false;
    public static final int kBackRightAbsoluteEncoderID = 0;
    public static final double kAbsolutueMaxSpeeds = 0;
    public static final double kRelativeMaxSpeeds = 0;
    }
}
