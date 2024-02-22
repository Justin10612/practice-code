// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeBackCommand extends Command {
  /** Creates a new IntakeBackCommand. */
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  public IntakeBackCommand(IntakeSubsystem _intakeSubsystem, ShooterSubsystem _shooterSubsystem) {
    this.intakeSubsystem = _intakeSubsystem;
    this.shooterSubsystem = _shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.detectNote() == true){
      intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakePrimetivePosition);
      intakeSubsystem.shouldturn(false);
      shooterSubsystem.shouldTransportTurn(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
