// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new Visionsubsystem. */
  private final PhotonCamera photonVision;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private int targetID;


  public PhotonVisionSubsystem() {
    // Camera
    photonVision = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  }

  public int getTargetID(){
    return targetID;
  }

  public Transform3d getTargetPose(){
   return target.getBestCameraToTarget();
  }

  public boolean hasTarget(){
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = photonVision.getLatestResult();
    target = result.getBestTarget();
    SmartDashboard.putBoolean("Photon/hasTarget", result.hasTargets());
    if(hasTarget()){
      double botXValue = getTargetPose().getX();
      double botYValue = getTargetPose().getY();
      double botZValue = getTargetPose().getRotation().getAngle();
      SmartDashboard.putNumber("Photon/TargetID", getTargetID());
      SmartDashboard.putNumber("Photon/botXValue", botXValue);
      SmartDashboard.putNumber("Photon/botYValue", botYValue);
      SmartDashboard.putNumber("Photon/botZValue", botZValue);
    }
  }
}