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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeMotor; 
  private final CANSparkMax intakePivotMotor;
  
  private final RelativeEncoder intakePivotEncoder;

  private final CANcoder intakPivotCancoder;
  private final CANcoderConfiguration intakePivotCancoderCofig;

  private final PIDController intakePivotPID;
  private final ArmFeedforward intakePivotFeedforward;

  private double PivotAngleSetpoint = IntakeConstants.kIntakeIdleAngle;

  public IntakeSubsystem() {
    intakePivotPID = new PIDController(0.003, 0, 0);
    intakePivotFeedforward = new ArmFeedforward(0, 0, 0);
    // CAN coder
    intakPivotCancoder = new CANcoder(IntakeConstants.kIntakePivotCancoderID);
    intakePivotCancoderCofig = new CANcoderConfiguration();
    // Motor Controllers
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
    intakePivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotMotorID, MotorType.kBrushless);
    //Relative Encoder
    intakePivotEncoder = intakePivotMotor.getEncoder();
    
    intakeMotor.restoreFactoryDefaults();
    intakePivotMotor.restoreFactoryDefaults();

    intakeMotor.setInverted(false);
    intakePivotMotor.setInverted(false);

    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakePivotMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
    intakePivotMotor.burnFlash();

    intakePivotCancoderCofig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    intakePivotCancoderCofig.MagnetSensor.MagnetOffset = IntakeConstants.kIntakePivotCancoderOffset;
    intakePivotCancoderCofig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    intakPivotCancoder.getConfigurator().apply(intakePivotCancoderCofig);
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
    intakeMotor.setVoltage(IntakeConstants.kIntakingMotorVoltage);
  }

  public void WheelEject(){
    intakeMotor.setVoltage(IntakeConstants.kEjectingMotorVoltage);
  }

  public void WheelStop(){
    intakeMotor.setVoltage(0);
  }
  @Override
  public void periodic() {
    // Display Data
    SmartDashboard.putNumber("intakeAngle", getAngle());
    // Intake PID Calculation
    double PidOutput = intakePivotPID.calculate(getAngle(), PivotAngleSetpoint);
    // PID deadband
    PidOutput = Constants.setMaxOutput(PidOutput, IntakeConstants.kPivotMaxOutput);
    // Intake Feedforward Calclation
    double FeedforwardOutput = intakePivotFeedforward.calculate(Math.toRadians(getAngle()), intakePivotEncoder.getVelocity()*2*Math.PI/60)/12;
    // Implement
    if(Math.abs(intakePivotPID.getPositionError())<2){
      intakePivotMotor.set(0);
    }else{
      intakePivotMotor.set(PidOutput);
    }
    // intakePivotMotor.set(PidOutput);
  }
}
