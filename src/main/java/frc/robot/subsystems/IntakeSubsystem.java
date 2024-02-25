// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeTurnMotor; 
  private final CANSparkMax intakePivotMotor; 

  private final CANcoder intakPivotCancoder;
  private final CANcoderConfiguration intakeShaftCancoderCofig;

  private final PIDController intakePivotPID = new PIDController(0.005, 0, 0);

  private double PivotAngleSetpoint = IntakeConstants.kIntakeIdleAngle;
  private final double intakeShaftCancoderOffset = 0.266;

  public IntakeSubsystem() {
    // CAN coder
    intakPivotCancoder = new CANcoder(45);
    intakeShaftCancoderCofig = new CANcoderConfiguration();
    // Motor Controllers
    intakeTurnMotor = new CANSparkMax(13, MotorType.kBrushless);
    intakePivotMotor = new CANSparkMax(27, MotorType.kBrushless);
    intakeTurnMotor.restoreFactoryDefaults();
    intakePivotMotor.restoreFactoryDefaults();

    intakeTurnMotor.setInverted(false);
    intakePivotMotor.setInverted(false);

    intakeTurnMotor.setIdleMode(IdleMode.kCoast);
    intakePivotMotor.setIdleMode(IdleMode.kBrake);

    intakeTurnMotor.burnFlash();
    intakePivotMotor.burnFlash();

    intakeShaftCancoderCofig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    intakeShaftCancoderCofig.MagnetSensor.MagnetOffset = intakeShaftCancoderOffset;
    intakeShaftCancoderCofig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    intakPivotCancoder.getConfigurator().apply(intakeShaftCancoderCofig);
  }

  public void setIntakeShaftSetpoint(double angleSetpoint){
    PivotAngleSetpoint = angleSetpoint;
  }

  public double getAngle(){
    return intakPivotCancoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public void setIdleAngle(){
    PivotAngleSetpoint = IntakeConstants.kIntakeIdleAngle;
  }

  public void setIntakeAngle(){
    PivotAngleSetpoint = IntakeConstants.kIntakingAngle;
  }

  public void WheelIntaking(){
    intakeTurnMotor.setVoltage(IntakeConstants.kIntakingMotorVoltage);
  }

  public void WheelEject(){
    intakeTurnMotor.setVoltage(IntakeConstants.kEjectingMotorVoltage);
  }

  public void WheelStop(){
    intakeTurnMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // Display Data
    SmartDashboard.putNumber("intakeAngle", getAngle());
    // Intake PID Calculation
    double PidOutput = intakePivotPID.calculate(getAngle(), PivotAngleSetpoint);
    PidOutput = Constants.setMaxOutput(PidOutput, IntakeConstants.kPivotMaxOutput);
    // PID deadband
    if(Math.abs(intakePivotPID.getPositionError())<2){
      intakePivotMotor.set(0);
    }else{
      intakePivotMotor.set(PidOutput);
    }
  }
}
