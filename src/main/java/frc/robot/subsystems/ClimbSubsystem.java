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
  private final CANSparkMax climbLeftMotot;

  private final DigitalInput climbRightLimitSwitch;
  private final DigitalInput climbLeftLimitSwitch;

  private final RelativeEncoder climbRightEncoder;
  private final RelativeEncoder climbLeftEncoder;

  private double climbRightPosition;
  private double climbLeftPosition;
  public ClimbSubsystem() {
    // Motor Controllers
    climbRightMotor = new CANSparkMax(ClimbConstants.kClimbRightMotorID, MotorType.kBrushless);
    climbLeftMotot = new CANSparkMax(ClimbConstants.kClimbLeftMotorID, MotorType.kBrushless);
    //LimitSwitch
    climbRightLimitSwitch = new DigitalInput(ClimbConstants.kRightLimitSwitchPort);
    climbLeftLimitSwitch = new DigitalInput(ClimbConstants.kLeftLimitSwitchPort);
    //RelativeEncoder
    climbRightEncoder = climbRightMotor.getEncoder();
    climbLeftEncoder = climbLeftMotot.getEncoder();

    climbRightMotor.restoreFactoryDefaults();
    climbLeftMotot.restoreFactoryDefaults();

    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbLeftMotot.setIdleMode(IdleMode.kBrake);


    climbRightMotor.setInverted(false);
    climbLeftMotot.setInverted(true);

    climbRightMotor.burnFlash();
    climbLeftMotot.burnFlash();

  }

  public void resetEncoder(){
    climbRightMotor.getEncoder().setPosition(0);
    climbLeftMotot.getEncoder().setPosition(0);
  }
  public void rightturn(double value){//152
    // if(climbRightPosition >= 139){
    //   if(value > 0){
    //     climbRightMotor.setVoltage(0);
    //   }
    //   else{
    //     climbRightMotor.setVoltage(value*12);
    //   }
    // }
    // else if(climbRightLimitSwitch.get()){
    //   if(value < 0){
    //     climbRightMotor.setVoltage(0);
    //   }
    //   else{
    //     climbRightMotor.setVoltage(value*12);
    //   }
    // }
    // else{
    //   climbRightMotor.setVoltage(value*12);
    // }
    climbRightMotor.setVoltage(value*12);
  }

  public void leftTurn(double value){
    // if(climbLeftPosition >= 152){
    //   if(value > 0){
    //     climbLeftMotot.setVoltage(0);
    //   }
    //   else{
    //     climbLeftMotot.setVoltage(value*12);
    //   }
    // }
    // else if(climbLeftLimitSwitch.get()){
    //   if(value < 0){
    //     climbLeftMotot.setVoltage(0);
    //   }
    //   else{
    //     climbLeftMotot.setVoltage(value*12);
    //   }
    // }
    // else{
    //   climbLeftMotot.setVoltage(value*12);
    // }
    climbLeftMotot.setVoltage(value*12);
  }

  @Override
  public void periodic() {
    // climbLeftMotot.set(0);
    climbRightPosition = climbRightEncoder.getPosition();
    climbLeftPosition = climbLeftEncoder.getPosition();
    SmartDashboard.putNumber("climbRightPosition", climbRightPosition);
    SmartDashboard.putNumber("climbLeftPosition", climbLeftPosition);
    SmartDashboard.putBoolean("rightLimit", climbRightLimitSwitch.get());
    
    if(climbLeftLimitSwitch.get()){
      climbLeftEncoder.setPosition(0);
    }
    if(climbRightLimitSwitch.get()){
      climbRightEncoder.setPosition(0);
    }
  }

}
