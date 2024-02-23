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
  private double shooterturnSetpoint;
  private double shooterShaftPIDOutput;
  private double shooterTurnPIDOutput;
  private double shooterShaftAngle;
  private double shooterShaftSetpoint = 0;
  private double shooterShaftErrorvalue;

  private boolean shouldShafTurn = false;
  private boolean shouldTransportTurn = false;
  private boolean needShoot;

  private final double shooterCancoderOffset = 0;
  private boolean transportreverse = true;
  public static boolean haveNote = false;

  private boolean noteING;

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

  public void shoot(){
    if(haveNote){
      if(shooterTurnSpeed >= shooterturnSetpoint-500){
        shooterTurnMotor.setVoltage(shooterturnSetpoint/5676*12);
        shouldTransportTurn(true);
        transportMotorSpeed = 4;
        needShoot = true;
      }
      else{
        shooterTurnMotor.setVoltage(shooterturnSetpoint/5676*12);
        shouldTransportTurn(false);
      }
    }
    else{
      shooterTurnMotor.setVoltage(0);
      shouldTransportTurn(false);
      needShoot = false;
    }
  }

  public void isNoteIn(boolean noteIn){
    noteING = noteIn;
  }

  public void shooterMotorstop(){
    shooterTurnMotor.set(0);
    shouldTransportTurn(false);
  }

  //for auto
  public void shooterMotorTurn(double speed){
    shooterturnSetpoint = speed;
    shooterTurnMotor.setVoltage(speed);
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

  public void shouldTransportTurn(boolean shouldTurn){
    shouldTransportTurn = shouldTurn;
  }


  @Override
  public void periodic() {
    if(haveNote && noteING){
      shouldTransportTurn(false);
      noteING = false;
    }
    haveNote = NoteGet.get();
    SmartDashboard.putNumber("shooterSpeed", shooterTurnSpeed);
    shooterTurnSpeed = shooterTurnEncoder.getVelocity();
    shooterTransportMotor.setInverted(transportreverse);
    if(shouldTransportTurn){
      shooterTransportMotor.setVoltage(transportMotorSpeed);
    }
    else{
      transportMotorStop();
    }
    SmartDashboard.putBoolean("haveNote", haveNote);
    // SmartDashboard.putNumber("shooterSpeed", shooterTurnSpeed);
    // SmartDashboard.putNumber("time", time.get());

  }
}
