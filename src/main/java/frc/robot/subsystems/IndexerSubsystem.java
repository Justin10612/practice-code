// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final DigitalInput BottomLimitSwitch;


  public IndexerSubsystem() {
    indexerMotor = new TalonFX(ShooterConstants.kIndexerMotorID); 
    indexerMotor.setInverted(true);
    // Note Sensor
    BottomLimitSwitch = new DigitalInput(ShooterConstants.kBottomLimitSwitchPort);
  }

  /**
   * When Shooter Velocity reach the setpoint, feed Note into it.
   */
  public void FeedWhenReady_SPEAKER(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerFastVolt);
  }
  public void FeedWhenReady_AMP(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerNormalVolt);
  }
  public void Intaking(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerNormalVolt);
  }
  public void Ejecting(){
    indexerMotor.setVoltage(ShooterConstants.kIndexerNormalVolt);
  }
  public void StopIndexerMotor(){
    indexerMotor.setVoltage(0);
  }

  /**
   * @return True when there is a Note.
   */
  public boolean getBottomSwitchState(){
    return !BottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Indexer/BottomSW", getBottomSwitchState());
    SmartDashboard.putNumber("Indexer/MotorTemp", indexerMotor.getDeviceTemp().getValueAsDouble());
  }
}
