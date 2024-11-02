// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax climbRightMotor;
  private final CANSparkMax climbLeftMotor;

  private final DigitalInput climberRightLimitSW;
  private final DigitalInput climberLeftLimitSW;

  private final RelativeEncoder climbRightEncoder;
  private final RelativeEncoder climbLeftEncoder;

  private boolean leftFlag = true;
  private boolean rightFlag = true;

  public ClimbSubsystem() {
    // Motor Controllers
    climbRightMotor = new CANSparkMax(ClimbConstants.kClimbRightMotorID, MotorType.kBrushless);
    climbLeftMotor = new CANSparkMax(ClimbConstants.kClimbLeftMotorID, MotorType.kBrushless);
    //LimitSwitch
    climberRightLimitSW = new DigitalInput(ClimbConstants.kRightLimitSwitchPort);
    climberLeftLimitSW = new DigitalInput(ClimbConstants.kLeftLimitSwitchPort);
    //RelativeEncoder
    climbRightEncoder = climbRightMotor.getEncoder();
    climbLeftEncoder = climbLeftMotor.getEncoder();

    climbRightMotor.restoreFactoryDefaults();
    climbLeftMotor.restoreFactoryDefaults();

    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbLeftMotor.setIdleMode(IdleMode.kBrake);

    climbRightMotor.setInverted(false);
    climbLeftMotor.setInverted(true);

    climbRightMotor.burnFlash();
    climbLeftMotor.burnFlash();

  }

  public void resetEncoder(){
    climbRightEncoder.setPosition(0);
    climbLeftEncoder.setPosition(0);
  }

  public double getLeftPosition(){
    return climbLeftEncoder.getPosition();
  }

  public double getRightPosition(){
    return climbRightEncoder.getPosition();
  }

  /** 若壓到的話是True 
  */
  public boolean getLeftLimitState(){
    return !climberLeftLimitSW.get();
  }

  /** 若壓到的話是True 
  */
  public boolean getRightLimitState(){
    return !climberRightLimitSW.get();
  }

  public void setRightMotor(double value){
    /* After */
    if(value >0){
      if(getRightPosition() >= 110) climbRightMotor.setVoltage(0);
      else climbRightMotor.setVoltage(value*12);
    }else{
      if(getLeftLimitState()) climbRightMotor.setVoltage(0);
      else climbRightMotor.setVoltage(value*12);
    }
  }

  public void setLeftMotor(double value){
    /* After */
    if(value >0){
      if(getLeftPosition() >= 108) climbLeftMotor.setVoltage(0);
      else climbLeftMotor.setVoltage(value*12);
    }else{
      if(getRightLimitState()) climbLeftMotor.setVoltage(0);
      else climbLeftMotor.setVoltage(value*12);
    }
  }

  public void StopMotors(){
    climbLeftMotor.setVoltage(0);
    climbRightMotor.setVoltage(0);
  }

  public void leftClimbOut(){
    if(getLeftPosition() <= 50){
      climbLeftMotor.setVoltage(9.6);
    }else{
      climbLeftMotor.setVoltage(0);
    }
  }

  public void rightClimbOut(){
    if(getRightPosition() <= 50){
      climbRightMotor.setVoltage(9.6);
    }else{
      climbRightMotor.setVoltage(0);
    }
  }

  public void leftClimbBack(){
    if(getRightLimitState()){
      climbLeftMotor.setVoltage(0);
    }else{
      climbLeftMotor.setVoltage(-9.6);
    }
  }
  public void rightClimbBack(){
    if(getLeftLimitState()){
      climbRightMotor.setVoltage(0);
    }else{
      climbRightMotor.setVoltage(-9.6);
    }
  }

  @Override
  public void periodic() {
    // ClimberPose
    SmartDashboard.putNumber("Climber/Right", getRightPosition());
    SmartDashboard.putNumber("Climber/Left", getLeftPosition());
    // LimitSwitch
    SmartDashboard.putBoolean("Climber/RightLimit", getRightLimitState());
    SmartDashboard.putBoolean("Climber/LeftLimit", getLeftLimitState());
    /* Encoder Zeroing */
    if(getRightLimitState() && leftFlag){
      for(int i=0;i<4;i++) climbLeftEncoder.setPosition(0);
      leftFlag = false;
    }else if(!getRightLimitState()){
      leftFlag = true;
    }
    /* Another Encoder Zeroing */
    if(getLeftLimitState() && rightFlag){
      for(int i=0;i<4;i++) climbRightEncoder.setPosition(0);
      rightFlag = false;
    }else if(!getLeftLimitState() && rightFlag){
      rightFlag = true;
    }
  }

}
