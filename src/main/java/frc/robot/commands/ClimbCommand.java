// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import static frc.robot.RobotContainer.*;

public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  private final ClimbSubsystem climbSubsystem;
  public ClimbCommand(ClimbSubsystem _climbSubsystem) {
    this.climbSubsystem = _climbSubsystem;
    addRequirements(climbSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.rightturn(armJoystick.getRightY()*-0.6);
    climbSubsystem.leftTurn(armJoystick.getLeftY()*-0.6);
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
