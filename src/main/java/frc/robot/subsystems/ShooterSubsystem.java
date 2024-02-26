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

  private final PIDController shooterTurnPID;
  
  private final RelativeEncoder shooterTurnEncoder;

  private final DigitalInput noteGet;

  private double shooterVoltageSetpoint = ShooterConstants.shooterSpeakerVoltageSetpoint;
  private double shooterRPMSetpoint = ShooterConstants.shooterSpeakerRPMSetpoint;
  public ShooterSubsystem() {
    // Motor Controllers
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(ShooterConstants.kIndexerMotorID, MotorType.kBrushless);

    shooterTurnEncoder = shooterMotor.getEncoder();
    //ID Sensor
    noteGet = new DigitalInput(ShooterConstants.kIRPort);

    shooterTurnPID = new PIDController(0, 0, 0);

    shooterMotor.restoreFactoryDefaults();
    indexerMotor.restoreFactoryDefaults();

    shooterMotor.setInverted(true);
    indexerMotor.setInverted(true);

    shooterMotor.setIdleMode(IdleMode.kCoast);
    indexerMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor.burnFlash();
    indexerMotor.burnFlash();
  }

  public double getShooterMotorTurnSpeed(){
    return shooterTurnEncoder.getVelocity();
  }

  public void shoot(){
    if(getShooterMotorTurnSpeed() >= shooterRPMSetpoint - 800){
        shooterMotor.setVoltage(shooterVoltageSetpoint);
        indexerMotor.setVoltage(4);
      }
      else{
        shooterMotor.setVoltage(shooterVoltageSetpoint);
        indexerMotor.setVoltage(0);
      }
  }

  public void InverseShoot(){
    shooterMotor.setVoltage(-shooterVoltageSetpoint);
    if(getShooterMotorTurnSpeed() <= shooterRPMSetpoint + 800){
        indexerMotor.setVoltage(4);
      }
      else{
        indexerMotor.setVoltage(0);
      }
  }

    public void shooterMotorTurn(double voltage, double rpm){
    shooterVoltageSetpoint = voltage;
    shooterRPMSetpoint = rpm;
    shooterMotor.setVoltage(voltage);
  }

  public void shooterMotorstop(){
    shooterMotor.setVoltage(0);
    indexerMotor.setVoltage(0);
  }

  public void Feeding(){
    indexerMotor.setVoltage(4);
  }

  public void Ejecting(){
    indexerMotor.setVoltage(-4);
  }

  public void StopTranportMotor(){
    indexerMotor.setVoltage(0);
  }

  
  public boolean detectNote(){
    return noteGet.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterSpeed", getShooterMotorTurnSpeed());
    SmartDashboard.putNumber("shooterMotorTurnSetpoint", shooterVoltageSetpoint);
  }
}
