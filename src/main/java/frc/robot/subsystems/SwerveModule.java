// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
 private final CANSparkMax fowardMotor;
 private final CANSparkMax turningMotor;
 private final CANCoder fowardEncoder;
 private final CANCoder turningEncoder;
 private final edu.wpi.first.wpilibj.AnalogInput absoluteEncoder;
 private final Boolean absoluteEncoderReversed;
 private final Double absoluteEncoderOffset;
 private final PIDController turningPidController;

  //constructor for each swerve
  public SwerveModule(
  int turningMotorid,
  int fowardMotorid, 
  int fowardEncoderID, 
  int turningEncoderid, 
  boolean frontMotorInverted,
  boolean turningMotorInverted, 
  Double absoluteEncoderOffset, 
  boolean absoluteEncoderInverted,
  int absoluteEncoderID,
  String moduleIndentifier)
  
  {
    fowardMotor = new CANSparkMax(fowardMotorid, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorid, MotorType.kBrushless);
    turningMotor.setInverted(turningMotorInverted);
    fowardMotor.setInverted(frontMotorInverted);
    


    absoluteEncoder = new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderID);
    this.absoluteEncoderReversed = absoluteEncoderInverted;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    

    fowardEncoder = new CANCoder(fowardEncoderID);
    turningEncoder = new CANCoder(turningEncoderid);
    //Config to be expanded in the future(maybe own subystem)
    CANCoderConfiguration turningConfig = new CANCoderConfiguration();
    turningConfig.sensorCoefficient = SwerveModuleConstants.kTurnEncoder2Radians;
    turningConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    CANCoderConfiguration Fowardconfig = new CANCoderConfiguration();
    turningConfig.sensorCoefficient = SwerveModuleConstants.kDriveEncoder2Meter;
    turningConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    fowardEncoder.configAllSettings(Fowardconfig);
    turningEncoder.configAllSettings(turningConfig);
    


    this.turningPidController = 
    new PIDController(
      SwerveModuleConstants.kPTurning, 
      SwerveModuleConstants.kITurning, 
      SwerveModuleConstants.KDTurning
    );
   
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

    
  
  }
  //Absolute Encoder to be researched further
  public double getAbsoluteEncoder(){
     double angle = absoluteEncoder.getVoltage()/ 
     RobotController.getVoltage5V();
     angle = angle * 2 * Math.PI;
     angle -= absoluteEncoderOffset;
    return angle * (absoluteEncoderReversed? -1.0: 1.0);
  }

  public double getFowardEncoderVelocity(){
    return fowardEncoder.getVelocity();
  }

  public double getTurningEncoderVelocity(){
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
    return new SwerveModuleState(getFowardEncoderVelocity(), 
    new Rotation2d(getTurningEncoderVelocity()));
  }
  //takes in a state and sets each module to the state 
  public void setDesiredState (SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < .01){

      stop();
      return;
    }
    state = 
    SwerveModuleState.optimize(state, getState().angle);
    
    fowardMotor.set(
    state.speedMetersPerSecond/SwerveModuleConstants.KFfowardspeed);

    turningMotor.set(
     turningPidController.calculate(
     getTurningPosition(), state.angle.getRadians()
  ));
  }
  //stops all motors
  public void stop(){
    fowardMotor.set( 0.0);
    turningMotor.set( 0.0);
  }


  

  
  }

