// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetectionSubsystem extends SubsystemBase {
  /** Creates a new NoteDetectionSubsystem. */
  private final PhotonCamera photonVision;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  public NoteDetectionSubsystem() {
    // Camera
    photonVision = new PhotonCamera("Microsoft_LifeCam_HD-3000_TrackNote");
  }

  public boolean hasTarget(){
    return result.hasTargets();
  }

  public double getNoteAngle(){
    if(hasTarget()){
      return target.getYaw();
    }else{
      return 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = photonVision.getLatestResult();
    target = result.getBestTarget();
    if(hasTarget()){
      SmartDashboard.putNumber("NoteDetection/NoteAngle", getNoteAngle());
    }
  }
}
