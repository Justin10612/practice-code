// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAMP extends Command {
  /** Creates a new AimAMPCommand. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  // PID Controller
  private final PIDController yMovePID;
  private final PIDController xMovePID;
  private final PIDController turnPID;
  // Target Value 
  private double botXValue;
  private double botYValue;
  private double botZValue; 
  //
  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  public AimAMP(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;
    // PID Controller
    xMovePID = new PIDController(VisionConstants.XmoveKp, VisionConstants.XmoveKi, VisionConstants.XmoveKd);
    yMovePID = new PIDController(VisionConstants.YmoveKp, VisionConstants.YmoveKi, VisionConstants.YmoveKd);
    turnPID = new PIDController(VisionConstants.ZRotationKp, VisionConstants.ZRotationKi, VisionConstants.ZRotationKd);
    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    LEDConstants.aimingAMP = true;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    double[] output = {0, 0, 0};
    // PID calculation
    if(m_PhotonVisionSubsystem.hasTarget()){
      // Get Measurement
      botXValue = m_PhotonVisionSubsystem.getTargetPose().getX();
      botYValue = m_PhotonVisionSubsystem.getTargetPose().getY();
      botZValue = m_PhotonVisionSubsystem.getTargetPose().getRotation().getAngle();
      // PID calculation
      double xMovePIDOutput = xMovePID.calculate(botXValue, VisionConstants.AMP_Setpoint[0]);
      double yMovePIDOutput = yMovePID.calculate(botYValue, VisionConstants.AMP_Setpoint[1]);
      double zTurnPIDOutput = -turnPID.calculate(botZValue, VisionConstants.AMP_Setpoint[2]);
      // Bounded Output
      output[0] = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
      output[1] = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
      output[2] = Constants.setMaxOutput(zTurnPIDOutput, maxTurnPIDOutput);
      // Impl
      m_SwerveSubsystem.drive(output[0], output[1], output[2], false);
      // LED
      LEDConstants.haveApriltag = true;
      LEDConstants.aimReadyAMP = Math.abs(xMovePID.getPositionError())<2 && Math.abs(yMovePID.getPositionError())<2 && Math.abs(turnPID.getPositionError())<2;
      LEDConstants.LEDFlag = true;
    }else{
      // Impl
      m_SwerveSubsystem.drive(0, 0, 0, false);
      // LED
      LEDConstants.haveApriltag = true;
      LEDConstants.LEDFlag = true;
    }
    // Print Output
    SmartDashboard.putNumber("Photon/X_AimPid", output[0]);
    SmartDashboard.putNumber("Photon/Y_AimPid", output[1]);
    SmartDashboard.putNumber("Photon/Z_AimPid", output[2]);
  }

  @Override
  public void end(boolean interrupted) {
    LEDConstants.aimingAMP = true;
    LEDConstants.aimReadyAMP = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LEDConstants.aimReadyAMP;
  }
}
