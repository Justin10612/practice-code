// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private double setpoint;
  private boolean intakeNeedTurn;
  private boolean shooterTransportNeedTurn;
  public IntakeCommand(IntakeSubsystem _intakeSubsystem, ShooterSubsystem _shooterSubsystem, double _setpoint, boolean _intakeNeedTurn, boolean _shooterTransportNeedTurn) {
    this.intakeSubsystem = _intakeSubsystem;
    this.shooterSubsystem = _shooterSubsystem;
    this.setpoint = _setpoint;
    this.intakeNeedTurn = _intakeNeedTurn;
    this.shooterTransportNeedTurn = _shooterTransportNeedTurn;
    addRequirements(intakeSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.getintakeShaftSetpoint(setpoint);
    intakeSubsystem.shouldturn(intakeNeedTurn);
    shooterSubsystem.shouldTransportTurn(shooterTransportNeedTurn);
    shooterSubsystem.transportMotorTurn();
    shooterSubsystem.isNoteIn(intakeNeedTurn);
    if(shooterTransportNeedTurn == false){
      shooterSubsystem.transportMotorStop();
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
