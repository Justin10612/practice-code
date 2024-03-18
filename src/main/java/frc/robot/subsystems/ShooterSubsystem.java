// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;

  private final RelativeEncoder shooterEncoder;

  private double RpmSetpoint = 0;

  public ShooterSubsystem() {
    // Motor Controllers
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.burnFlash();
  }

  /**
   * You have to put it in execute()
   */
  public void EnableShooter(double targetVoltage, double targetRPM){
    RpmSetpoint = targetRPM;
    // Implement
    shooterMotor.setVoltage(targetVoltage);
    SmartDashboard.putNumber("shooterRPMSetpoint", targetRPM);
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
    return getShooterSpeed()>=RpmSetpoint;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Measure", getShooterSpeed());
    SmartDashboard.putBoolean("Shooter/", achievedTargetSpeed());
    SmartDashboard.putNumber("Shooter/Output",shooterMotor.getAppliedOutput());
  }
}
