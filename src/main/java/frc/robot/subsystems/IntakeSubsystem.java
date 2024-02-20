// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeTurnMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax intakeShaftMotor = new CANSparkMax(27, MotorType.kBrushless);

  private final CANcoder intakeShaftCancoder = new CANcoder(45);
  private final CANcoderConfiguration intakeShaftCancoderCofig = new CANcoderConfiguration();

  private final PIDController intaleShaftPID = new PIDController(0.005, 0, 0);
  private final PIDController intakeTurnPID = new PIDController(0, 0, 0);

  private final RelativeEncoder intakeTurnEncoder = intakeTurnMotor.getEncoder();

  private double intakeShaftAngle;
  private double intakeTurnSpeedSetpoint = 2800;
  private double intakeShaftMaxPIDOutput = 0.1;
  private double intakeTurnSpeed;
  private double intakeShaftPIDOutput;
  private double intakeTurnPIDOutput;
  private double intakeShaftSetpoint = 12;
  private final double intakeShaftCancoderOffset = 0.266;
  private double intakeShaftErrorValue;
  private boolean turn;
  public IntakeSubsystem() {
    intakeTurnMotor.restoreFactoryDefaults();
    intakeShaftMotor.restoreFactoryDefaults();

    intakeTurnMotor.setInverted(false);
    intakeShaftMotor.setInverted(false);

    intakeTurnMotor.setIdleMode(IdleMode.kCoast);
    intakeShaftMotor.setIdleMode(IdleMode.kBrake);

    intakeTurnMotor.burnFlash();
    intakeShaftMotor.burnFlash();

    intakeShaftCancoderCofig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    intakeShaftCancoderCofig.MagnetSensor.MagnetOffset = intakeShaftCancoderOffset;
    intakeShaftCancoderCofig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    intakeShaftCancoder.getConfigurator().apply(intakeShaftCancoderCofig);
  }

  public void getintakeShaftSetpoint(double angleSetpoint){
    intakeShaftSetpoint = angleSetpoint;
  }

  public void shouldturn(boolean shouldTurn){
    turn = shouldTurn;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeAngle", intakeShaftAngle);
    intakeShaftAngle = intakeShaftCancoder.getAbsolutePosition().getValueAsDouble()*360;
    intakeTurnSpeed = intakeTurnEncoder.getVelocity();
    intakeShaftErrorValue = intakeShaftSetpoint - intakeShaftAngle;

    intakeShaftPIDOutput = intaleShaftPID.calculate(intakeShaftAngle, intakeShaftSetpoint);
    intakeTurnPIDOutput = intakeTurnPID.calculate(intakeTurnSpeed, intakeTurnSpeedSetpoint);
    intakeShaftPIDOutput = Constants.setMaxOutput(intakeShaftPIDOutput, intakeShaftMaxPIDOutput);
    
    //Motor move
    if(turn){
      intakeTurnMotor.setVoltage(intakeTurnSpeedSetpoint/5676*12 + intakeTurnPIDOutput);
    }
    else{
      intakeTurnMotor.setVoltage(0);
    }
    
    if(Math.abs(intakeShaftErrorValue) > 2){
      intakeShaftMotor.set(intakeShaftPIDOutput);
    }
    else{
      intakeShaftMotor.set(0);
    }
    if(intakeShaftErrorValue < 0){
      intakeShaftMaxPIDOutput = 0.1;
    }
    else{
      intakeShaftMaxPIDOutput = 0.15;
    }

  }
}
