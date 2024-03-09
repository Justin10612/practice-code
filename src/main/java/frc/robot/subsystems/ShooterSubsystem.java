// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;
  private final TalonFX indexerMotor;
  private final PIDController shooterPID;
  private final RelativeEncoder shooterEncoder;
  private final DigitalInput BottonLimitSwitch;
  private final DigitalInput TopLimitSwitch;

  private double Voltage_Setpoint = ShooterConstants.kShooterSpeakerVoltageSetpoint;
  private double RPM_Setpoint = ShooterConstants.kShooterSpeakerRPMSetpoint;

  public ShooterSubsystem() {
    // Motor Controllers
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    indexerMotor = new TalonFX(ShooterConstants.kIndexerMotorID);
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();
    // Note Sensor
    BottonLimitSwitch = new DigitalInput(ShooterConstants.kLowLimitSwitchPort);
    TopLimitSwitch = new DigitalInput(ShooterConstants.kUpLimitSwitchPort);

    shooterPID = new PIDController(ShooterConstants.kShooterKp, ShooterConstants.kShooterKi, ShooterConstants.kShooterKd);

    shooterMotor.restoreFactoryDefaults();

    shooterMotor.setInverted(true);
    indexerMotor.setInverted(true);

    shooterMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor.burnFlash();
  }

  /**
   * 給定馬達的目標電壓以及轉速，驅動Shooter馬達轉動。
   */
  public void EnableShooter(double targetVoltage, double targetRPM){
    // Without PID
    Voltage_Setpoint = targetVoltage;
    RPM_Setpoint = targetRPM;
    shooterMotor.setVoltage(targetVoltage);
  }

  /**
   * 給定馬達的目標電壓以及轉速，透過PID控制Shooter馬達轉動。
   * 要將它放在execute()中執行
   */
  public void EnableShooterPID(double targetVoltage, double targetRPM){
    // With PID
    Voltage_Setpoint = targetVoltage;
    RPM_Setpoint = targetRPM;
    // PID cal
    double pidOutput = shooterPID.calculate(getShooterSpeed(), targetRPM);
    pidOutput = Math.min(Math.max(-1, pidOutput), 1); // Output is limited between 1 and -1. Unit:volt
    // Implement
    shooterMotor.setVoltage(targetVoltage + pidOutput);
  }

  /**
   * When Shooter Velocity reach the setpoint, feed Note into it.
   */
  public void FeedWhenReady(){
    if(getShooterSpeed() >= RPM_Setpoint - 800){
      NormalFeeding();
    }else{
      StopIndexerMotor();
      EnableShooter(Voltage_Setpoint, RPM_Setpoint); //有需要寫句嗎
    }
  }

  public void StopMotors(){
    shooterMotor.setVoltage(0);
    indexerMotor.setVoltage(0);
  }

  public void NormalFeeding(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerNormalVolt);
  }
  public void SlowFeeding(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerSlowlVolt);
  }

  public void Ejecting(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerNormalVolt);
  }

  public void StopIndexerMotor(){
    indexerMotor.setVoltage(0);
  }

  /**
   * @return True when there is a Note.
   */
  public boolean getTopSwitchState(){
    return !TopLimitSwitch.get();
  }

  /**
   * @return True when there is a Note.
   */
  public boolean getBottonSwitchState(){
    return !BottonLimitSwitch.get();
  }

  /**
   * @return Flywheel Motor RPM
   */
  public double getShooterSpeed(){
    return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterMeasure", getShooterSpeed());
    SmartDashboard.putNumber("ShooterSetspeed", RPM_Setpoint);
    SmartDashboard.putBoolean("TopSW", getTopSwitchState());
    SmartDashboard.putBoolean("BottonSW", getBottonSwitchState());
  }
}
