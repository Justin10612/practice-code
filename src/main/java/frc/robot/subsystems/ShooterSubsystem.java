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

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterTurnMotor = new CANSparkMax(33, MotorType.kBrushless);
  // private final CANSparkMax shooterShafMotor = new CANSparkMax(48, MotorType.kBrushed);
  private final CANSparkMax shooterTransportMotor = new CANSparkMax(12, MotorType.kBrushless);

  // private final CANcoder shooterShaftCancoder = new CANcoder(54);
  private final CANcoderConfiguration shooterShaftCancoderCofig = new CANcoderConfiguration();

  // private final PIDController shooterShaftPID = new PIDController(0, 0, 0);
  private final PIDController shooterTurnPID = new PIDController(0, 0, 0);
  
  private final RelativeEncoder shooterTurnEncoder = shooterTurnMotor.getEncoder();

  private final DigitalInput NoteGet = new DigitalInput(0);
  private final Timer time = new Timer();

  private double transportMotorSpeed = 4;
  private double shooterTurnSpeed;
  private double shooterShaftPIDOutput;
  private double shooterTurnPIDOutput;
  private double shooterShaftAngle;
  private double shooterShaftSetpoint = 0;
  private double shooterShaftErrorvalue;

  private boolean shouldShafTurn = false;
  private boolean shouldTransportTurn = false;

  private final double shooterCancoderOffset = 0;
  private boolean transportreverse = true;
  public static boolean haveNote = false;

  public ShooterSubsystem() {
    time.reset();
    shooterTurnMotor.restoreFactoryDefaults();
    // shooterShafMotor.restoreFactoryDefaults();
    shooterTransportMotor.restoreFactoryDefaults();

    shooterTurnMotor.setInverted(true);
    // shooterShafMotor.setInverted(false);
    shooterTransportMotor.setInverted(true);

    shooterTurnMotor.setIdleMode(IdleMode.kCoast);
    // shooterShafMotor.setIdleMode(IdleMode.kBrake);
    shooterTransportMotor.setIdleMode(IdleMode.kCoast);

    shooterTurnMotor.burnFlash();
    // shooterShafMotor.burnFlash();
    shooterTransportMotor.burnFlash();

    shooterShaftCancoderCofig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    shooterShaftCancoderCofig.MagnetSensor.MagnetOffset = shooterCancoderOffset;
    shooterShaftCancoderCofig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // shooterShaftCancoder.getConfigurator().apply(shooterShaftCancoderCofig);
  }

  public void shooterMotorTurn(double shooterSpeed){
    if(haveNote){
      if(shooterTurnSpeed >= shooterSpeed-500){
        shooterTurnMotor.setVoltage(shooterSpeed);
        shouldTransportTurn(true);
        transportMotorSpeed = 4;
      }
      else{
        shooterTurnMotor.setVoltage(shooterSpeed);
        shouldTransportTurn(false);
      }
    }
    else{
      shooterTurnMotor.setVoltage(0);
      shouldTransportTurn(false);
    }
    // shooterTurnMotor.setVoltage(9.6);
    // shouldTransportTurn = true;
    
  }

  public void shooterMotorstop(){
    shooterTurnMotor.set(0);
    shouldTransportTurn(false);
  }

  public void transportMotorTurn(double speed){
    shooterTransportMotor.setVoltage(speed);
  }

  public void transportMotorStop(){
    shooterTransportMotor.setVoltage(0);
  }

  public void transportMotorTurn(){
    transportMotorSpeed = 4;
  }

  public void getShooterShaftsetpoint(double angleSetpoint){
    shooterShaftSetpoint = angleSetpoint;
  }

  public boolean detectNote(){
    return haveNote;
  }

  public void transportMotorReverse(){
    transportMotorSpeed = -4;
  }

  // public void shaftTurn(double value){
  //   shouldShafTurn = true;
  //   shooterShaftPIDOutput = value;
  // }

  // public void shaftStop(){
  //   shouldShafTurn = false;
  // }

  public void shouldTransportTurn(boolean shouldTurn){
    shouldTransportTurn = shouldTurn;
  }


  @Override
  public void periodic() {
    if(haveNote){
      time.start();
    }
    else{
      time.stop();
    }
    if(haveNote && shooterTurnSpeed <= 1000){
      shouldTransportTurn(false);
    }
    haveNote = NoteGet.get();
    SmartDashboard.putNumber("shooterSpeed", shooterTurnSpeed);
    // shooterShaftAngle = shooterShaftCancoder.getAbsolutePosition().getValueAsDouble();
    shooterTurnSpeed = shooterTurnEncoder.getVelocity();
    shooterTransportMotor.setInverted(transportreverse);
    // shooterShaftErrorvalue = shooterShaftSetpoint - shooterShaftAngle;

    if(shouldTransportTurn){
      transportMotorTurn(transportMotorSpeed);
    }
    else{
      transportMotorStop();
    }
    // shooterShaftPIDOutput = shooterShaftPID.calculate(shooterShaftAngle, shooterShaftSetpoint);

    // if(shouldShafTurn){
    //   shooterShafMotor.set(shooterShaftPIDOutput);
    // }
    // else{
    //   shooterShafMotor.set(0);
    // }
    SmartDashboard.putBoolean("haveNote", haveNote);
    SmartDashboard.putNumber("shooterSpeed", shooterTurnSpeed);
    SmartDashboard.putNumber("time", time.get());

  }
}
