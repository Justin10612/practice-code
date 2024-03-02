// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new Visionsubsystem. */
  private final PhotonCamera photonLimelight;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private boolean hasTarget;

  private final PIDController yMovePID;
  private final PIDController xMovePID;
  private final PIDController turnPID;

  private final Optional<Alliance> alliance;

  private double yMovePIDOutput, xMovePIDOutput, turnPIDOutput;

  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  private double botXValue;
  private double botYValue;
  private double botZValue;
  private double xSetpoint;
  private double ySetpoint;
  private double zSetpoint;
  private int targetID;

  public PhotonVisionSubsystem() {
    // Camera
    photonLimelight = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    yMovePID = new PIDController(0.005, 0, 0);
    xMovePID = new PIDController(0.0030, 0, 0);
    turnPID = new PIDController(0.005, 0, 0);

    alliance = DriverStation.getAlliance();
  }
  public void getSetpoint(double[] setpoint){
    xSetpoint = setpoint[0];
    ySetpoint = setpoint[1];
    zSetpoint = setpoint[2];
  }

  public double getXSpeed(){
    return xMovePIDOutput;
  }

  public double getYSpeed(){
    return yMovePIDOutput;
  }

  public double getTurnSpeed(){
    return turnPIDOutput;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = photonLimelight.getLatestResult();
    target = result.getBestTarget();
    hasTarget = result.hasTargets();

    if(hasTarget){
      botXValue = result.getBestTarget().getBestCameraToTarget().getX();
      botYValue = result.getBestTarget().getBestCameraToTarget().getY();
      botZValue = result.getBestTarget().getBestCameraToTarget().getRotation().getAngle();
      targetID = target.getFiducialId();
    }
    else{
      botXValue = xSetpoint;
      botYValue = ySetpoint;
      botZValue = zSetpoint;
      targetID = 0;
    }

    if(alliance.get() == DriverStation.Alliance.Red){
      getSetpoint(AimConstants.redModeSelect(targetID));
    }
    else{
      getSetpoint(AimConstants.blueModeSelect(targetID));
    }

    yMovePIDOutput = yMovePID.calculate(botXValue, xSetpoint);
    xMovePIDOutput = xMovePID.calculate(botYValue, ySetpoint);
    turnPIDOutput = -turnPID.calculate(botZValue, zSetpoint);

    xMovePIDOutput = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
    yMovePIDOutput = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
    turnPIDOutput = Constants.setMaxOutput(turnPIDOutput, maxTurnPIDOutput);
   
    SmartDashboard.putNumber("photonZ", botZValue);
    SmartDashboard.putNumber("photonY", botYValue);
    SmartDashboard.putNumber("photonX", botXValue);
    SmartDashboard.putNumber("targetID", targetID);

    SmartDashboard.putNumber("xMovePIDOutput", xMovePIDOutput);
    SmartDashboard.putNumber("yMovePIDOutput", yMovePIDOutput);
    SmartDashboard.putNumber("turn", turnPIDOutput);
  }
}