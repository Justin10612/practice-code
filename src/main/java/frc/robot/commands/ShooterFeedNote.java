// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class ShooterFeedNote extends Command {
  /** Creates a new NoteShootSpeakerCommand. */
  private final IndexerSubsystem m_IndexerSubsystem;

  public ShooterFeedNote(IndexerSubsystem indexerSubsystem) {
    this.m_IndexerSubsystem = indexerSubsystem;
    addRequirements(m_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 把它放在這，因為他需要時時監看是否到達目標轉速。
    m_IndexerSubsystem.FeedWhenReady();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IndexerSubsystem.StopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
