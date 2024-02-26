// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  private final ClimbSubsystem m_climbSubsystem;
  private final Boolean climberEnable;
  private final DoubleSupplier m_leftInput;
  private final double m_rightInput;

  public ClimbCommand(ClimbSubsystem climbSubsystem, DoubleSupplier leftInput, DoubleSupplier RightInput) {
    this.m_climbSubsystem = climbSubsystem;
    this.
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armJoystick.leftBumper().getAsBoolean()){
      m_climbSubsystem.setRightMotor(armJoystick.getRightY()*-0.8);
      m_climbSubsystem.setLeftMotor(armJoystick.getLeftY()*-0.8);
    }
    else{
      m_climbSubsystem.StopMotors();
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
