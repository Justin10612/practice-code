// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExcludeNoteCommand extends Command {
  /** Creates a new ExcludeNoteCommand. */
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private double intakeShaftSetpoint;
  private boolean intakeNeedTurn;
  public ExcludeNoteCommand(IntakeSubsystem _intakeSubsysyem, ShooterSubsystem _shooterSubsystem, double _intakeShaftSetpoint, boolean _intakeNeedTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = _intakeSubsysyem;
    this.shooterSubsystem = _shooterSubsystem;
    this.intakeShaftSetpoint = _intakeShaftSetpoint;
    this.intakeNeedTurn = _intakeNeedTurn;
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.getintakeShaftSetpoint(intakeShaftSetpoint);
    intakeSubsystem.shouldturn(intakeNeedTurn);
    shooterSubsystem.transportMotorReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakePrimetivePosition);
    intakeSubsystem.shouldturn(false);
    shooterSubsystem.shooterMotorstop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
