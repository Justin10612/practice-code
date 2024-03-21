// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNote extends Command {
  /** Creates a new TrackNote. */
  private final SwerveSubsystem m_SwerveSubsystem;
  private final LimeLightSubsystem m_LimeLightSubsystem;
  /* PID Controller */
  private final PIDController m_pid;
  private double pidOutput;
  private double inputValue;

  public TrackNote(SwerveSubsystem swerveSubsystem, LimeLightSubsystem lightSubsystem) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_LimeLightSubsystem = lightSubsystem;
    m_pid = new PIDController(0.02, 0, 0);
    addRequirements(m_LimeLightSubsystem, m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidOutput = 0;
    inputValue = 0;
    // LED
    LEDConstants.trackingNote = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inputValue = m_LimeLightSubsystem.getNoteX();
    pidOutput = m_pid.calculate(inputValue, 0);
    pidOutput = Math.min(Math.max(pidOutput, -0.4), 0.4);
    if(m_LimeLightSubsystem.hasNote()){
      LEDConstants.hasNoteInSight = true;
      LEDConstants.LEDFlag = true;
      if(Math.abs(m_pid.getPositionError())>5){
        m_SwerveSubsystem.drive(0, 0, pidOutput, false);
      }else{
        m_SwerveSubsystem.drive(0.2, 0, 0, false);
      }
    }else{
      LEDConstants.hasNoteInSight = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stopModules();
    LEDConstants.trackingNote = false;
    LEDConstants.hasNoteInSight = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LEDConstants.hasNote;
  }
}
