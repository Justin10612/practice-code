// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_intakeSubsystem, m_shooterSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeAngle();
    m_intakeSubsystem.WheelIntaking();
    m_shooterSubsystem.NormalFeeding();
  }

  @Override
  public void execute(){
    if(m_shooterSubsystem.getBottonSwitchState()){
      m_shooterSubsystem.SlowFeeding();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIdleAngle();
    m_intakeSubsystem.WheelStop();
    m_shooterSubsystem.StopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterSubsystem.getTopSwitchState();
  }
}
