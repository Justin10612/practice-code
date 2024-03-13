// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectNoteIdlePose extends Command {
  private final IndexerSubsystem m_IndexerSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  public EjectNoteIdlePose(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;
    addRequirements(m_IntakeSubsystem, m_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIdleAngle();
    m_IntakeSubsystem.WheelEject();
    m_IndexerSubsystem.Intaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIdleAngle();
    m_IntakeSubsystem.WheelStop();
    m_IndexerSubsystem.StopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
