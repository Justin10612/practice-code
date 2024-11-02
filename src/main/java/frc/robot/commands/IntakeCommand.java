// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;
    addRequirements(m_IntakeSubsystem, m_IndexerSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeAngle();
    m_IntakeSubsystem.WheelIntaking();
    m_IndexerSubsystem.Intaking();
    LEDConstants.intaking = true;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIdleAngle();
    m_IntakeSubsystem.WheelStop();
    m_IndexerSubsystem.StopIndexerMotor();
    // LED
    if(m_IndexerSubsystem.getBottomSwitchState()){
      LEDConstants.hasNote = true;
    }else{
      LEDConstants.hasNote = false;
    }
    LEDConstants.intaking = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IndexerSubsystem.getBottomSwitchState();
  }
}
