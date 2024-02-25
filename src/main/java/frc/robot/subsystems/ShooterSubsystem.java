// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterTurnMotor;
  private final CANSparkMax shooterIndaxerMotor;

  private final PIDController shooterTurnPID = new PIDController(0, 0, 0);
  
  private final RelativeEncoder shooterTurnEncoder;

  private final DigitalInput noteGet = new DigitalInput(4);

  private double shooterTurnSpeed;

  private final double shooterCancoderOffset = 0;
  private double shooterVoltageSetpoint = ShooterConstants.shooterSpeakerVoltageSetpoint;
  private double shooterRPMSetpoint = ShooterConstants.shooterSpeakerRPMSetpoint;
  public static boolean haveNote = false;
  public ShooterSubsystem() {
    shooterTurnMotor = new CANSparkMax(33, MotorType.kBrushless);
    shooterIndaxerMotor = new CANSparkMax(12, MotorType.kBrushless);
    shooterTurnEncoder = shooterTurnMotor.getEncoder();
    shooterTurnMotor.restoreFactoryDefaults();
    // shooterShafMotor.restoreFactoryDefaults();
    shooterIndaxerMotor.restoreFactoryDefaults();

    shooterTurnMotor.setInverted(true);
    // shooterShafMotor.setInverted(false);
    shooterIndaxerMotor.setInverted(true);

    shooterTurnMotor.setIdleMode(IdleMode.kCoast);
    // shooterShafMotor.setIdleMode(IdleMode.kBrake);
    shooterIndaxerMotor.setIdleMode(IdleMode.kCoast);

    shooterTurnMotor.burnFlash();
    // shooterShafMotor.burnFlash();
    shooterIndaxerMotor.burnFlash();
  }

  public void shoot(){
    if(shooterTurnSpeed >= shooterRPMSetpoint - 800){
        shooterTurnMotor.setVoltage(shooterVoltageSetpoint);
        shooterIndaxerMotor.setVoltage(4);
      }
      else{
        shooterTurnMotor.setVoltage(shooterVoltageSetpoint);
        shooterIndaxerMotor.setVoltage(0);
      }
  }

  public void InverseShoot(){
    shooterTurnMotor.setVoltage(-shooterVoltageSetpoint);
    if(shooterTurnSpeed <= shooterRPMSetpoint + 800){
        shooterIndaxerMotor.setVoltage(4);
      }
      else{
        shooterIndaxerMotor.setVoltage(0);
      }
  }

  public void shooterMotorstop(){
    shooterTurnMotor.setVoltage(0);
    shooterIndaxerMotor.setVoltage(0);
  }

  public void shooterMotorTurn(double voltage, double rpm){
    shooterVoltageSetpoint = voltage;
    shooterRPMSetpoint = rpm;
    shooterTurnMotor.setVoltage(voltage);
  }

  public boolean detectNote(){
    return haveNote;
  }

  public void Feeding(){
    shooterIndaxerMotor.setVoltage(4);
  }

  public void Ejecting(){
    shooterIndaxerMotor.setVoltage(-4);
  }

  public void StopTranportMotor(){
    shooterIndaxerMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    haveNote = false;
    shooterTurnSpeed = shooterTurnEncoder.getVelocity();
    SmartDashboard.putNumber("shooterSpeed", shooterTurnSpeed);
    SmartDashboard.putNumber("shooterMotorTurnSetpoint", shooterVoltageSetpoint);
    SmartDashboard.putBoolean("haveNote", haveNote);

  }
}
