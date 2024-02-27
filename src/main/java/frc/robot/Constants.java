// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static double DeadBandLimit(double input, double deadBand){
    return Math.abs(input)>deadBand ? input : 0;
  }
  public static class OperatorConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;
    public static final double kJoystickDeadBand = 0.1;
  }
  public static final double robotLength = Units.inchesToMeters(25.5);
  public static final double robotWidth = Units.inchesToMeters(25.5);

  public static double setMaxOutput(double value, double maxOutput){
    return Math.min(maxOutput, Math.max(value, -maxOutput));
  }
  public static final class IntakeConstants{
    //CanCoder ID
    public static final int kIntakePivotCancoderID = 45;
    //Cancoder Setting
    public static final double kIntakePivotCancoderOffset = 0.266;
    // Pivot Angle
    public static final double kIntakingAngle = -110;
    public static final double kIntakeIdleAngle = 12;
    //Motor ID
    public static final int kIntakePivotMotorID = 13;
    public static final int kIntakeMotorID = 27;
    // Motor Setting 
    public static final double kPivotMaxOutput = 0.12;
    public static final double kIntakingMotorVoltage = 5;
    public static final double kEjectingMotorVoltage = -6;
  }

  public static final class ShooterConstants{
    //Motor ID
    public static final int kShooterMotorID = 33;
    public static final int kIndexerMotorID = 13;
    //IR ID
    public static final int kIRPort = 4;
    //Motor Setting
    public static final double shooterSpeakerVoltageSetpoint = 9.6;
    public static final double shooterAMP_VoltageSetpoint = 3;
    public static final double shooterSpeakerRPMSetpoint = 4000;
    public static final double shooterAMP_RPMSetpoint = 1000;
  }

  public static final class ClimbConstants{
    //Motor ID
    public static final int kClimbRightMotorID = 31;
    public static final int kClimbLeftMotorID = 8;
    //limitSwitch Port
    public static final int kRightLimitSwitchPort = 1;
    public static final int kLeftLimitSwitchPort = 2;
  }

  public static final class ApriltagConstants{
    public static final int blueSpeakerCenterID = 7;
    public static final int blueSpeakerLeftID = 8;
    public static final int blueAMPID = 9;
    public static final int bluesourceRightID = 1;
    public static final int blueSourceLeftID = 2;

    public static final int redSpeakerCenterID = 4;
    public static final int redSpeakerRightID = 3;
    public static final int redAMPID = 5;
    public static final int redSourceRightID = 9;
    public static final int redSourceLeftID = 10;

    public static final double speakerHeight = 204.5;//cm
    public static final double armHeight = 28.16;//cm

    public static final double limelightToArmDistance = 0;
    public static final double armAndEndEffectorAngle = 120.0;

    public static double[] redModeSelect(int targetID){
      double[] setpoint;
      switch (targetID) {
        case redAMPID:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        case redSourceLeftID:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        case redSpeakerCenterID,redSpeakerRightID , redSourceRightID:
          setpoint = new double[]{0,0,0,0}; 
        default:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        
      }
      return setpoint;
    }
    public static double[] blueModeSelect(int targetID){
      double[] setpoint;
      switch (targetID) {
        case blueAMPID:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        case blueSourceLeftID:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        case blueSpeakerCenterID, blueSpeakerLeftID, bluesourceRightID:
          setpoint = new double[]{0,0,0,0}; 
        default:
          setpoint = new double[]{0, 0, 0, 0};
          break;
        
      }
      return setpoint;
    }

  }

  public static final class SwerveModuleConstants{
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 1/6.75;
    public static final double turningMotorGearRatio = 1.0/(150/7);
    public static final double driveEncoderRot2Meter = driveMotorGearRatio*Math.PI*wheelDiameter;
    public static final double turningEncoderRot2Rad = turningMotorGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;
    public static final double turningMotorkP = 0.015;
    public static final double maxDriveMotorSpeed = 4.0;
  }
  public static final class SwerveConstants{
    public static final int leftFrontDriveID = 19;
    public static final int leftFrontTurningID = 17;
    public static final int rightFrontDriveID = 11;
    public static final int rightFrontTurningID = 24;  
    public static final int leftRearDriveID = 15;
    public static final int leftRearTurningID = 22;
    public static final int rightRearDriveID = 16;
    public static final int rightRearTurningID = 26;

    public static final int leftFrontCANCoderID = 44;
    public static final int rightFrontCANCoderID = 43;
    public static final int leftRearCANCoderID = 42;
    public static final int rightRearCANCoderID = 41;

    public static final boolean leftFrontdriveMotorReversed = true;
    public static final boolean leftFrontTurningMotorReversed = true;
    public static final boolean rightFrontDriveMotorReversed = true;
    public static final boolean rightfrontTurningMotorReversed = true;
    public static final boolean leftRearDriveMotorreversed = false;
    public static final boolean leftRearTurningMotorReversed = true;
    public static final boolean rightRearDriveMotorReversed = false;
    public static final boolean rightRearTurningMotorReversed = true;

    public static final double leftFrontOffset = 0.129;
    public static final double rightFrontOffset = -0.355;
    public static final double leftRearOffset = -0.345;
    public static final double rightRearOffset = 0.185;
    
    public static double joysickValue(double value, double mineOutput){
      if(Math.abs(value) < mineOutput){
        return 0;
      }
      else{
        return value;
      }
    }

    public static final int gyroID = 33;
    //front left, front right, rear left, rear right
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(robotLength/2, robotWidth/2), 
      new Translation2d(robotLength/2, -robotWidth/2), 
      new Translation2d(-robotLength/2, robotWidth/2),
      new Translation2d(-robotLength/2, -robotWidth/2)
  );
  }
}
