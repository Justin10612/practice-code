// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPreparingCommand extends Command {
  /** Creates a new ShooterPreparingCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private double motorVoltage;
  private double motorRPM;
  public ShooterPreparingCommand(ShooterSubsystem _shooterSubsystem, double _motorVoltage, double _motorRPM) {
    this.shooterSubsystem = _shooterSubsystem;
    this.motorRPM = _motorRPM;
    this.motorVoltage = _motorVoltage;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shooterMotorTurn(motorVoltage, motorRPM);
  }

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
