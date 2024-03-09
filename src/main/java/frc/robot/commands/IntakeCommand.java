// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;
    addRequirements(m_intakeSubsystem, m_IndexerSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeAngle();
    m_intakeSubsystem.WheelIntaking();
    m_IndexerSubsystem.NormalFeeding();
  }

  @Override
  public void execute(){
    if(m_IndexerSubsystem.getBottonSwitchState()){
      m_IndexerSubsystem.StopIndexerMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIdleAngle();
    m_intakeSubsystem.WheelStop();
    m_IndexerSubsystem.StopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IndexerSubsystem.getBottonSwitchState();
  }
}
