// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAMP extends Command {
  /** Creates a new AimAMPCommand. */
  private final PhotonVisionSubsystem m_photonVisionSuubsystem;
  private final SwerveSubsystem m_swerveSubsystem;

  public AimAMP(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.m_photonVisionSuubsystem = photonVisionSubsystem;
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_photonVisionSuubsystem, m_swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(m_photonVisionSuubsystem.hasTarget()){
      // Get Setpoint
      double[] targetSetpoint = VisionConstants.getTargetSetpoint(m_photonVisionSuubsystem.getTargetID());    
      // Aiming Calculation
      double[] driveOutput = m_photonVisionSuubsystem.AimingTargetPID(
        targetSetpoint[0],
        targetSetpoint[1], 
        targetSetpoint[2]);
      // Move Drivebase
      m_swerveSubsystem.driveRobotRelative(driveOutput[0], driveOutput[1], driveOutput[2]);
    }else{
      // LED blinking or Something...
      m_swerveSubsystem.driveRobotRelative(0, 0, 0);
    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
