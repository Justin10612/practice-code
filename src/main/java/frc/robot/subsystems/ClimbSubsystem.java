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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax climbRightMotor = new CANSparkMax(31, MotorType.kBrushless);
  private final CANSparkMax climbLeftMotot = new CANSparkMax(8, MotorType.kBrushless);

  private final DigitalInput climbRightLimitSwitch = new DigitalInput(0);
  private final DigitalInput climbLeftLimitSwitch = new DigitalInput(3);

  private final RelativeEncoder climbRightEncoder = climbRightMotor.getEncoder();
  private final RelativeEncoder climbLeftEncoder = climbLeftMotot.getEncoder();

  private double climbRightPosition;
  private double climbLeftPosition;
  public ClimbSubsystem() {
    climbRightMotor.restoreFactoryDefaults();
    climbLeftMotot.restoreFactoryDefaults();

    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbLeftMotot.setIdleMode(IdleMode.kBrake);


    climbRightMotor.setInverted(false);
    climbLeftMotot.setInverted(true);

    climbRightMotor.burnFlash();
    climbLeftMotot.burnFlash();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void resetEncoder(){
    climbRightMotor.getEncoder().setPosition(0);
    climbLeftMotot.getEncoder().setPosition(0);
  }
  public void rightturn(double value){//152
    if(climbRightPosition >= 139){
      if(value > 0){
        climbRightMotor.setVoltage(0);
      }
      else{
        climbRightMotor.setVoltage(value*12);
      }
    }
    else if(climbRightLimitSwitch.get()){
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

  public void leftTurn(double value){
    if(climbLeftPosition >= 152){
      if(value > 0){
        climbLeftMotot.setVoltage(0);
      }
      else{
        climbLeftMotot.setVoltage(value*12);
      }
    }
    else if(climbLeftLimitSwitch.get()){
      if(value < 0){
        climbLeftMotot.setVoltage(0);
      }
      else{
        climbLeftMotot.setVoltage(value*12);
      }
    }
    else{
      climbLeftMotot.setVoltage(value*12);
    }
  }

  @Override
  public void periodic() {
    climbRightPosition = climbRightEncoder.getPosition();
    climbLeftPosition = climbLeftEncoder.getPosition();
    
    if(climbLeftLimitSwitch.get()){
      climbLeftEncoder.setPosition(0);
    }
    if(climbRightLimitSwitch.get()){
      climbRightEncoder.setPosition(0);
    }
  }

}
