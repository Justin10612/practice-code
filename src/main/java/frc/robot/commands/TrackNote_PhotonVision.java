// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNote_PhotonVision extends Command {
  /** Creates a new TrackNote_PhotonVision. */
  private final NoteDetectionSubsystem m_NoteDetectionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private final PIDController m_PidController;
  private double pidOutput;
  private double inputValue;
  public TrackNote_PhotonVision(NoteDetectionSubsystem noteDetectionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_NoteDetectionSubsystem = noteDetectionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    m_PidController = new PIDController(0.02, 0, 0);
    addRequirements(m_NoteDetectionSubsystem, m_SwerveSubsystem);
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
    inputValue = m_NoteDetectionSubsystem.getNoteAngle();
    pidOutput = m_PidController.calculate(inputValue, 0);
    pidOutput = Math.min(Math.max(pidOutput, -0.4), 0.4);
    if(m_NoteDetectionSubsystem.hasTarget()){
      LEDConstants.hasNoteInSight = true;
      LEDConstants.LEDFlag = true;
      if(Math.abs(m_PidController.getPositionError())>5){
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
