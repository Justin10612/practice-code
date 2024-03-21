// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAMP extends Command {
  /** Creates a new ShooterPreparingFrAMPCommand. */
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  private final boolean m_isAuto;
  private final BooleanSupplier m_feedBtnFunc;
  
  public ShootAMP(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, BooleanSupplier btnFunc, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;
    this.m_feedBtnFunc = btnFunc;
    this.m_isAuto = isAuto;
    addRequirements(m_shooterSubsystem, m_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDConstants.prepAMP = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean feedBtn = m_feedBtnFunc.getAsBoolean();
    if(m_isAuto){
      if(m_shooterSubsystem.achievedTargetSpeed()){
        m_IndexerSubsystem.FeedWhenReady_AMP();
        LEDConstants.speedReadyAMP = true;
        LEDConstants.LEDFlag = true;
      } 
    }else{
      m_shooterSubsystem.EnableShooter(ShooterConstants.kShooterAMP_VoltageSetpoint, ShooterConstants.kShooterAMP_RPMSetpoint);
      if(feedBtn && m_shooterSubsystem.achievedTargetSpeed())
        m_IndexerSubsystem.FeedWhenReady_AMP();
      else if (m_shooterSubsystem.achievedTargetSpeed()) {
        LEDConstants.speedReadyAMP = true;
        LEDConstants.LEDFlag = true;
      }else {
        LEDConstants.speedReadyAMP = false;
        LEDConstants.prepAMP = true;
        LEDConstants.LEDFlag = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooterMotor();
    m_IndexerSubsystem.StopIndexerMotor();
    if(m_IndexerSubsystem.getBottomSwitchState()){
      LEDConstants.hasNote = true;
    }else{
      LEDConstants.hasNote = false;
    }
    LEDConstants.speedReadyAMP = false;
    LEDConstants.prepAMP = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
