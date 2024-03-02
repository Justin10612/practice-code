// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAMP extends Command {
  /** Creates a new AimAMPCommand. */
  private final PhotonVisionSubsystem m_photonVisionSuubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  public AimAMP(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_photonVisionSuubsystem = photonVisionSubsystem;
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_photonVisionSuubsystem, m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_photonVisionSuubsystem.getXSpeed();
    double ySpeed = m_photonVisionSuubsystem.getYSpeed();
    double zSpeed = m_photonVisionSuubsystem.getTurnSpeed();
    m_swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
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
