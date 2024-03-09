// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new Visionsubsystem. */
  private final PhotonCamera photonVision;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private int targetID;

  private final PIDController yMovePID;
  private final PIDController xMovePID;
  private final PIDController turnPID;  
  
  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  public PhotonVisionSubsystem() {
    // Camera
    photonVision = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    
    xMovePID = new PIDController(VisionConstants.XmoveKp, VisionConstants.XmoveKi, VisionConstants.XmoveKd);
    yMovePID = new PIDController(VisionConstants.YmoveKp, VisionConstants.YmoveKi, VisionConstants.YmoveKd);
    turnPID = new PIDController(VisionConstants.ZRotationKp, VisionConstants.ZRotationKi, VisionConstants.ZRotationKd);
  }

  public int getTargetID(){
    return targetID;
  }

  public Transform3d getTargetPose(){
   return target.getBestCameraToTarget();
  }

  /**
   * @param xSetpoint 
   * @param ySetpoint
   * @param zSetpoint
   * @return [xMovePIDOutput, yMovePIDOutput, zTurnPIDOutput]
   */
  public double[] AimingTargetPID(double xSetpoint, double ySetpoint, double zSetpoint){
    double[] output = {0, 0, 0};
    // PID calculation
    if(hasTarget()){
      // Get Measurement
      double botXValue = getTargetPose().getX();
      double botYValue = getTargetPose().getY();
      double botZValue = getTargetPose().getRotation().getAngle();
      // PID calculation
      double xMovePIDOutput = xMovePID.calculate(botXValue, xSetpoint);
      double yMovePIDOutput = yMovePID.calculate(botYValue, ySetpoint);
      double zTurnPIDOutput = -turnPID.calculate(botZValue, zSetpoint);
      // Bounded Output
      output[0] = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
      output[1] = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
      output[2] = Constants.setMaxOutput(zTurnPIDOutput, maxTurnPIDOutput);
      // Print Output
      SmartDashboard.putNumber("X_AimPid", output[0]);
      SmartDashboard.putNumber("Y_AimPid", output[1]);
      SmartDashboard.putNumber("Z_AimPid", output[2]);
      return output;
    }else{
      return output;
    }
  }

  public boolean hasTarget(){
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = photonVision.getLatestResult();
    target = result.getBestTarget();

    if(hasTarget()){
      targetID = target.getFiducialId();
      SmartDashboard.putBoolean("HasTarget", hasTarget());
      SmartDashboard.putNumber("targetID", targetID);
      SmartDashboard.putNumber("TargetX", getTargetPose().getX());
      SmartDashboard.putNumber("TargetY", getTargetPose().getY());
      SmartDashboard.putNumber("TargetZ", getTargetPose().getRotation().getZ());
    }else{
      SmartDashboard.putBoolean("HasTarget", false);
    }
  }
}