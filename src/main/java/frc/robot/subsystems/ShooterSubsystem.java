// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;
 
  private final PIDController shooterPID;
  private final RelativeEncoder shooterEncoder;

  public ShooterSubsystem() {
    // Motor Controllers
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();

    shooterPID = new PIDController(ShooterConstants.kShooterKp, ShooterConstants.kShooterKi, ShooterConstants.kShooterKd);

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.burnFlash();
  }

  /**
   * You have to put it in execute()
   */
  public void EnableShooterPID(double targetVoltage, double targetRPM){
    // PID cal
    double pidOutput = shooterPID.calculate(getShooterSpeed(), targetRPM);
    pidOutput = Math.min(Math.max(-1, pidOutput), 1); // Output is limited between 1 and -1. Unit:volt
    // Implement
    shooterMotor.setVoltage(targetVoltage + pidOutput);
  }

  public void stopShooterMotor(){
    shooterMotor.set(0);
  }

  /**
   * @return Flywheel Motor RPM
   */
  public double getShooterSpeed(){
    return shooterEncoder.getVelocity();
  }
  public boolean achievedTargetSpeed(){
    return shooterPID.getPositionError()<100;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterMeasure", getShooterSpeed());
  }
}
