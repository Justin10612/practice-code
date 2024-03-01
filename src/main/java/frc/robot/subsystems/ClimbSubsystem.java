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

  public void setRightMotor(double value){//152
    if(getRightPosition() >= 139){
      if(value > 0){
        climbRightMotor.setVoltage(0);
      }
      else{
        climbRightMotor.setVoltage(value*12);
      }
    }
    else if(getRightLimitState()){
      if(value < 0){
        climbRightMotor.setVoltage(0);
      }
      else{
        climbRightMotor.setVoltage(value*12);
      }
    }
    else{
      climbRightMotor.setVoltage(value*12);
    }
  }

  public void setLeftMotor(double value){
    if(getLeftPosition() >= 152){
      if(value > 0){
        climbLeftMotor.setVoltage(0);
      }
      else{
        climbLeftMotor.setVoltage(value*12);
      }
    }
    else if(getLeftLimitState()){
      if(value < 0){
        climbLeftMotor.setVoltage(0);
      }
      else{
        climbLeftMotor.setVoltage(value*12);
      }
    }
    else{
      climbLeftMotor.setVoltage(value*12);
    }
  }

  public void StopMotors(){
    climbLeftMotor.setVoltage(0);
    climbRightMotor.setVoltage(0);
  }

  public void leftClimbOut(){
    if(getLeftPosition() <= 50){
      climbLeftMotor.setVoltage(9.6);
    }
    else{
      climbLeftMotor.setVoltage(0);
    }
  }

  public void rightClimbOut(){
    if(getRightPosition() <= 50){
      climbRightMotor.setVoltage(9.6);
    }
    else{
      climbRightMotor.setVoltage(0);
    }
  }

  public void leftClimbBack(){
    if(getLeftLimitState()){
      climbLeftMotor.setVoltage(0);
    }
    else{
      climbLeftMotor.setVoltage(-8);
    }
  }
  public void rightClimbBack(){
    if(getRightLimitState()){
      climbRightMotor.setVoltage(0);
    }
    else{
      climbRightMotor.setVoltage(-8);
    }
  }

  @Override
  public void periodic() {
    // ClimberPose
    SmartDashboard.putNumber("climbRightPosition", getRightPosition());
    SmartDashboard.putNumber("climbLeftPosition", getLeftPosition());
    // LimitSwitch
    SmartDashboard.putBoolean("Right Limit", getRightLimitState());
    SmartDashboard.putBoolean("Left Limit", getLeftLimitState());
    
    if(getLeftLimitState()){
      climbLeftEncoder.setPosition(0);
    }
    if(getRightLimitState()){
      climbRightEncoder.setPosition(0);
    }
  }

}
