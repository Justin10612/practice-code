// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  /* Robot Parameters */
  public static final double robotLength = Units.inchesToMeters(25.5);
  public static final double robotWidth = Units.inchesToMeters(25.5);
  public static final double kModuleDistance = 21*0.0254;
  /* ============
   *   Joystick
   * ============
   */
  public static class OperatorConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;
    public static final double kJoystickDeadBand = 0.1;
  }
  /* ==========
   *   Intake
   * ==========
   */
  public static final class IntakeConstants{
    //CanCoder ID
    public static final int kIntakePivotCancoderID = 45;
    //Cancoder Setting
    public static final double kIntakePivotCancoderOffset = 0.443;
    // Pivot Angle
    public static final double kIntakingAngle = -90;
    public static final double kIntakeIdleAngle = 1.58;
    //Motor ID
    public static final int kIntakePivotMotorID = 27;
    public static final int kIntakeMotorID = 32;
    // Motor Setting 
    public static final double kPivotMaxOutput = 0.12;
    public static final double kIntakingMotorVoltage = 8; //8
    public static final double kEjectingMotorVoltage = -6.5;
  }
  /* ===========
   *   Shooter
   * ===========
   */
  public static final class ShooterConstants{
    // Motor ID
    public static final int kShooterMotorID = 23;
    public static final int kIndexerMotorID = 13;
    // Limit Switch ID
    public static final int kUpLimitSwitchPort = 0;
    public static final int kBottomLimitSwitchPort = 3;
    // Motor Setting
    public static final double kShooterSpeakerVoltageSetpoint = 9.6;
    public static final double kShooterSpeakerVoltageSetpoint_Auto = 11;
    public static final double kShooterAMP_VoltageSetpoint = 3;
    public static final double kShooterStopVoltageSetpoint = 0;
    public static final double kShooterSpeakerRPMSetpoint = 4100;
    public static final double kShooterAMP_RPMSetpoint = 1000;
    public static final double kShooterStopRPMSetpoint = 9999;
    // Shooter PID Constants
    public static final double kShooterKp = 0; 
    public static final double kShooterKi = 0;
    public static final double kShooterKd = 0;
    // Indexer Speed
    public static final double kIndexerNormalVolt = 4;
    public static final double kIndexerFastVolt = 4;
    public static final double kIndexerSlowlVolt = 0;

    public static boolean shouldShoot = false;
  }
  /* ===========
   *   Climber
   * ===========
   */
  public static final class ClimbConstants{
    //Motor ID
    public static final int kClimbRightMotorID = 31;
    public static final int kClimbLeftMotorID = 8;
    //limitSwitch Port
    public static final int kRightLimitSwitchPort = 2;
    public static final int kLeftLimitSwitchPort = 1;
    // Encoder Value
    public static final int kRightEncoderLimit = 0;
    public static final int kLeftEncoderLimit = 0;
  }
  /* =======
   *   LED
   * =======
   */
  public static final class LEDConstants{
    public static final int kCANdleID = 46;
    public static final int kLedNum = 22;

    public static final int[] kHaveNoteRGBValue = {255, 0, 0};
    public static final int[] kIntakeTurningRGBValue = {20, 0, 0};
    public static final int[] kPrepToAMP_RGBValue = {0, 0, 0};
    public static final int[] kPrepToSpeakerRGBValue = {0, 0, 0};
    public static final int[] kShootAMP_RGBValue = {0, 0, 0};
    public static final int[] kShootSpeakerRGBValue = {0, 0, 0};
    public static final int[] kDisableRGBValue = {0, 0, 0};
  }
  /* ==========
   *   Vision
   * ==========
   */
  public static final class VisionConstants{
    // X Aiming PID
    public static final double XmoveKp = 0; 
    public static final double XmoveKi = 0;
    public static final double XmoveKd = 0;
    // Y Aiming PID
    public static final double YmoveKp = 0;
    public static final double YmoveKi = 0;
    public static final double YmoveKd = 0;
    // Z Aiming PID
    public static final double ZRotationKp = 0;
    public static final double ZRotationKi = 0;
    public static final double ZRotationKd = 0;
    // Target Setpoints
    public static final double[] TRAP_Setpoint = {1, 0, 0};
    public static final double[] AMP_Setpoint = {1, 0, 0};

    public static double[] getTargetSetpoint(int targetID){
      switch (targetID) {
        case AprilTagIDs.redAmpID:
        case AprilTagIDs.blueAmpID:
         return AMP_Setpoint;
        case AprilTagIDs.redTrapLeftID:
        case AprilTagIDs.redTrapCenterID:
        case AprilTagIDs.redTrapRightID:
        case AprilTagIDs.blueTrapLeftID:
        case AprilTagIDs.blueTrapCenterID:
        case AprilTagIDs.blueTrapRightID:
           return TRAP_Setpoint;
        default:
          return TRAP_Setpoint;
      }
    }
  }

  public static final class AprilTagIDs{
      public static final int blueSpeakerCenterID = 7;
      public static final int blueSpeakerLeftID = 8;
      public static final int blueAmpID = 9;
      public static final int bluesourceRightID = 1;
      public static final int blueSourceLeftID = 2;
      public static final int blueTrapLeftID = 15;
      public static final int blueTrapRightID = 16;
      public static final int blueTrapCenterID = 14; 

      public static final int redSpeakerCenterID = 4;
      public static final int redSpeakerRightID = 3;
      public static final int redAmpID = 5;
      public static final int redSourceRightID = 9;
      public static final int redSourceLeftID = 10;
      public static final int redTrapLeftID = 11;
      public static final int redTrapRightID = 12;
      public static final int redTrapCenterID = 13;
  }
  /* =================
   *   Swerve Module
   * =================
   */
  public static final class SwerveModuleConstants{
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 1/6.75;
    public static final double turningMotorGearRatio = 1.0/(150/7);
    public static final double driveEncoderRot2Meter = driveMotorGearRatio*Math.PI*wheelDiameter;
    public static final double turningEncoderRot2Rad = turningMotorGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;
    // Turn PID
    public static final double drivekP = 0.2;
    public static final double drivekI = 0;
    public static final double drivekD = 0;
    // Angle PID
    public static final double anglekP = 0.012;
    public static final double anglekI = 0;
    public static final double anglekD = 0.00025;
    // Max Speed
    public static final double maxDriveMotorSpeed = 4.5;
  }
  
  public static final class SwerveConstants{
    public static final int leftFrontDriveID = 19;
    public static final int leftFrontTurningID = 17;
    public static final int rightFrontDriveID = 29;
    public static final int rightFrontTurningID = 21;  
    public static final int leftRearDriveID = 15;
    public static final int leftRearTurningID = 22;
    public static final int rightRearDriveID = 16;
    public static final int rightRearTurningID = 26;

    public static final int leftFrontCANCoderID = 42;
    public static final int rightFrontCANCoderID = 43;
    public static final int leftRearCANCoderID = 44;
    public static final int rightRearCANCoderID = 41;

    public static final int gyroID = 33;

    public static final boolean leftFrontdriveMotorReversed = true;
    public static final boolean leftFrontTurningMotorReversed = true;
    public static final boolean rightFrontDriveMotorReversed = true;
    public static final boolean rightfrontTurningMotorReversed = true;
    public static final boolean leftRearDriveMotorreversed = true;
    public static final boolean leftRearTurningMotorReversed = true;
    public static final boolean rightRearDriveMotorReversed = true;
    public static final boolean rightRearTurningMotorReversed = true;

    public static final double leftFrontOffset = -0.0962;
    public static final double rightFrontOffset = -0.348;
    public static final double leftRearOffset = 0.3986;
    public static final double rightRearOffset = -0.31;

    /**
     * Drive base radius (distance from center to furthest module) 
     * Unit:meters */ 
    public static final double kDriveBaseRadius = 14.85 * 0.0254;
    
    //front left, front right, rear left, rear right
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2), 
      new Translation2d(kModuleDistance/2, -kModuleDistance/2), 
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );
  }
  /* =============
   *   Functions
   * =============
  */
  public static double DeadBandLimit(double input, double deadBand){
    return Math.abs(input)>deadBand ? input : 0;
  }

  public static double setMaxOutput(double value, double maxOutput){
    return Math.min(maxOutput, Math.max(value, -maxOutput));
  }
}
