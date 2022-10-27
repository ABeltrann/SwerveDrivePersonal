// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
  /** Creates a new SwerveModule. */
 private final TalonFX fowardMotor;
 private final TalonFX turningMotor;
 private final CANCoder fowardEncoder;
 private final CANCoder turningEncoder;
 private final edu.wpi.first.wpilibj.AnalogInput absoluteEncoder;
 private final Boolean absoluteEncoderReversed;
 private final Double absoluteEncoderOffset;


  public SwerveModule(int turningMotorid, int fowardMotorid, int fowardEncoderID, int turningEncoderid, TalonFXInvertType dInvertType, boolean turningMotorInverted, Double absoluteEncoderOffset, boolean absoluteEncoderInverted, int absoluteEncoderID, String moduleIndentifier ) {
    fowardMotor = new TalonFX(fowardMotorid);
    turningMotor = new TalonFX(turningMotorid);
    turningMotor.setInverted(turningMotorInverted);
    fowardMotor.setInverted(dInvertType);
    


    absoluteEncoder = new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderID);
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    fowardEncoder = new CANCoder(fowardEncoderID);
    turningEncoder = new CANCoder(turningEncoderid);
    CANCoderConfiguration turningConfig = new CANCoderConfiguration();
    turningConfig.sensorCoefficient = SwerveModuleConstants.kTurnEncoder2Radians;
    turningConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    CANCoderConfiguration Fowardconfig = new CANCoderConfiguration();
    turningConfig.sensorCoefficient = SwerveModuleConstants.kDriveEncoder2Meter;
    turningConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    fowardEncoder.configAllSettings(Fowardconfig);
    turningEncoder.configAllSettings(turningConfig);
    


    PIDController turningController = new PIDController(Constants.SwerveModuleConstants.kPTurning, 0, 0);
    turningController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

    
  
  }
  public double getAbsoluteEncoder(){
     double angle = absoluteEncoder.getVoltage()/ RobotController.getVoltage5V();
     angle = angle * 2 * Math.PI;
     angle -= absoluteEncoderOffset;
    return angle;
  }

  public double getFowardEncoder(){
    return fowardEncoder.getVelocity();
  }

  public double getTurningEncoder(){
    return turningEncoder.getVelocity();
  }
  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getFowardPosition(){
    return fowardEncoder.getPosition();
  }

  public void resetEncoders(){
   turningEncoder.setPosition(getAbsoluteEncoder());
   fowardEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getfo)
  }

  
  }

