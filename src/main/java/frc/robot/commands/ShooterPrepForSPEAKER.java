// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrepForSPEAKER extends Command {
  /** Creates a new ShooterPreparingCommand. */
  private final ShooterSubsystem m_shooterSubsystem;
  public ShooterPrepForSPEAKER(ShooterSubsystem shooterSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.EnableShooterPID(ShooterConstants.kShooterSpeakerVoltageSetpoint, ShooterConstants.kShooterSpeakerRPMSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooterMotor();
    m_shooterSubsystem.EnableShooter(ShooterConstants.kShooterStopVoltageSetpoint, ShooterConstants.kShooterStopRPMSetpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
