// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbManually extends Command {
  /** Creates a new ClimbCommand. */
  private final ClimbSubsystem m_climbSubsystem;
  private final DoubleSupplier m_leftFunc;
  private final DoubleSupplier m_rightFunc;

  public ClimbManually(ClimbSubsystem climbSubsystem, DoubleSupplier leftFunc, DoubleSupplier rightFunc) {
    this.m_climbSubsystem = climbSubsystem;
    this.m_leftFunc= leftFunc;
    this.m_rightFunc = rightFunc;
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Inputs
    double leftInputVal = m_leftFunc.getAsDouble();
    double rightInputVal = m_rightFunc.getAsDouble();
    // Output
    // 如果這裡加負號，是不是代表Subsystem那邊要SetInverted?
    m_climbSubsystem.setRightMotor(rightInputVal*-1);
    m_climbSubsystem.setLeftMotor(leftInputVal*-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
