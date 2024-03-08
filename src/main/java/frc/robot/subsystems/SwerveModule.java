package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
  // Motor Controller
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  // Relative Encoder
  private final RelativeEncoder driveEncoder;
  // private final RelativeEncoder turningEncoder;
  // PID controller
  private final PIDController turningPIDController;
  // CANcoder
  private final CANcoder absoluteEncoder;
  private final CANcoderConfiguration cancoderConfig;

  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffsetDegree){
    /* CAN coder Setting */  
    absoluteEncoder = new CANcoder(absoluteEncoderID);
    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDegree;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoder.getConfigurator().apply(cancoderConfig);
    /* Motor Controller */
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
    // Motor controller settings
    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.burnFlash();
    turningMotor.burnFlash();

    driveEncoder = driveMotor.getEncoder();
    // turningEncoder = turningMotor.getEncoder();

    turningPIDController = new PIDController(SwerveModuleConstants.turningMotorkP, 0, 0);
    turningPIDController.enableContinuousInput(-180, 180);
    /* Reset Encoder */
    resetEncoders();
  }


    public double getDrivePosition(){
      return driveEncoder.getPosition()*SwerveModuleConstants.driveEncoderRot2Meter;
    }
    public double getDriveVelocity(){
      return driveEncoder.getVelocity()*SwerveModuleConstants.driveEncoderRPM2MeterPerSec;
    }
    public double getTurningPosition(){
      return absoluteEncoder.getAbsolutePosition().getValue()*360;
    }
    // public double getTurnintEncoderPosition(){
      // return turningEncoder.getPosition()*SwerveModuleConstants.turningEncoderRot2Rad;
    // }
    // public double getTurningVelocity(){
    //   return turningEncoder.getVelocity()*SwerveModuleConstants.turningEncoderRPM2RadPerSec;
    // }
    public void resetEncoders(){
      driveEncoder.setPosition(0);
      // turningEncoder.setPosition(0);
    }
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    public void setDesiredState(SwerveModuleState state){
      state = SwerveModuleState.optimize(state, getState().angle);
      driveMotor.set(state.speedMetersPerSecond);
      turningMotor.set(turningPIDController.calculate(getState().angle.getDegrees(),state.angle.getDegrees()));
    }
    public void stopMotors(){
      driveMotor.set(0);
      turningMotor.set(0);
    }

    @Override
    public void periodic(){}
}