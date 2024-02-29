// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  private final NetworkTable limelight;
  private final NetworkTableEntry tx;
  private final PIDController turnPID;
  private double turnPIDOutput;
  public LimeLightSubsystem() {
    turnPID = new PIDController(0, 0, 0);

    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelight.getEntry("tx");
  }

  public double getNoteAngel(){
    return tx.getDouble(0);
  }

  public double getTurnPIDOutput(){
    return turnPIDOutput;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getNoteAngel();
    turnPIDOutput = turnPID.calculate(getNoteAngel(), 0);
    SmartDashboard.putNumber("NoteAngle", getNoteAngel());
  }
}
