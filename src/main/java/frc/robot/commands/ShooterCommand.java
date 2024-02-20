// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new shooterCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private double setpoint;
  public ShooterCommand(ShooterSubsystem _shooterSubsystem, double _setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = _shooterSubsystem;
    this.setpoint = _setpoint;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.getShooterShaftsetpoint(setpoint);
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
