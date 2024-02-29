// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final CANSparkMax indexerMotor;

  private final PIDController shooterPID;
  
  private final RelativeEncoder shooterEncoder;

  private final DigitalInput noteGet;

  private double shooterTurnPIDOutput;

  private double Voltage_Setpoint;
  private double RPM_Setpoint;

  public ShooterSubsystem() {
    // Motor Controllers
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(ShooterConstants.kIndexerMotorID, MotorType.kBrushless);
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();
    // Note Sensor
    noteGet = new DigitalInput(ShooterConstants.kLimitSwitchPort);

    shooterPID = new PIDController(0, 0, 0);

    shooterMotor.restoreFactoryDefaults();
    indexerMotor.restoreFactoryDefaults();

    shooterMotor.setInverted(true);
    indexerMotor.setInverted(true);

    shooterMotor.setIdleMode(IdleMode.kCoast);
    indexerMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor.burnFlash();
    indexerMotor.burnFlash();
  }

  /**
   * 給定馬達的目標電壓以及轉速，驅動Shooter馬達轉動。
   */
  public void EnableShooter(double targetVoltage, double targetRPM){
    // Without PID
    Voltage_Setpoint = targetVoltage;
    RPM_Setpoint = targetRPM;
    shooterMotor.setVoltage(targetVoltage + shooterTurnPIDOutput);
  }

  /**
   * When Shooter Velocity reach the setpoint, feed Note into it.
   */
  public void Shoot(){
    if(getShooterSpeed() >= RPM_Setpoint - 800){
      Feeding();
    }else{
      StopIndexerMotor();
    }
  }

  public void StopMotors(){
    shooterMotor.setVoltage(0);
    indexerMotor.setVoltage(0);
  }

  public void Feeding(){
    indexerMotor.setVoltage(4);
  }

  public void Ejecting(){
    indexerMotor.setVoltage(-4);
  }

  public void StopIndexerMotor(){
    indexerMotor.setVoltage(0);
  }

  /**
   * @return True when there is a Note.
   */
  public boolean detectNote(){
    return !noteGet.get();
  }

  /**
   * @return Flywheel Motor RPM
   */
  public double getShooterSpeed(){
    return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    shooterTurnPIDOutput = shooterPID.calculate(getShooterSpeed(), RPM_Setpoint);

    SmartDashboard.putNumber("shooterSpeed", getShooterSpeed());
    SmartDashboard.putNumber("Shooter Setpoint", Voltage_Setpoint);
  }
}
