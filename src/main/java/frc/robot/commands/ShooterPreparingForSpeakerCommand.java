// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPreparingForSpeakerCommand extends Command {
  /** Creates a new ShooterPreparingCommand. */
  private final ShooterSubsystem shooterSubsystem;
  public ShooterPreparingForSpeakerCommand(ShooterSubsystem _shooterSubsystem) {
    this.shooterSubsystem = _shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.shooterMotorTurn(ShooterConstants.shooterSpeakerVoltageSetpoint, ShooterConstants.shooterSpeakerRPMSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooterMotorstop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
