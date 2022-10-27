// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class  SwerveModuleConstants{
    public static final double kWheelDiameter = Units.inchesToMeters(1.5);
    public static final double kDriveGearRatio = 1/8.14;
    public static final double kTurnGearRatio = 1/12.8;
    public static final double kDriveEncoder2Meter = kWheelDiameter * Math.PI * kDriveGearRatio;
    public static final double kTurnEncoder2Radians = kTurnGearRatio * Math.PI * 2;
    public static final double kDriveEncoderMetersPerSecond = kDriveEncoder2Meter/60;
    public static final double kTurnEncoder2RadiansPerSecond = kTurnEncoder2Radians/60;
    public static final double kPTurning = .5;
    }
    public static final class SwerveSubsystemConstants{
    
    public static final double kDriveWidth = Units.inchesToMeters(25);
    public static final double kDriveLength = Units.inchesToMeters(25);
    //takes in the position of each module relative to the center of robot, order is Front Left, FrontRight, BackLeft, BackRight 
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kDriveWidth/2, -kDriveLength/2), new Translation2d(kDriveWidth/2, kDriveLength/2), new Translation2d(-kDriveWidth/2, -kDriveLength/2), new Translation2d(kDriveWidth/2, -kDriveLength/2));
    public static final Port kGyroId = null;
    }
}
