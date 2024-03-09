// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectNoteIdlePose extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  public EjectNoteIdlePose(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_ShooterSubsystem = shooterSubsystem;
    addRequirements(m_IntakeSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIdleAngle();
    m_IntakeSubsystem.WheelEject();
    m_ShooterSubsystem.NormalFeeding();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIdleAngle();
    m_IntakeSubsystem.WheelStop();
    m_ShooterSubsystem.StopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
