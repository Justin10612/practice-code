// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends Command {
  /** Creates a new ManualDriveCommand. */
  private final SwerveSubsystem m_swerveSubsystem;
  // Inputs
  private DoubleSupplier xSpeedFunc;
  private DoubleSupplier ySpeedFunc;
  private DoubleSupplier zSpeedFunc;
  private BooleanSupplier isSlowModeFunc;
  // Slew rate limiter
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;

  public ManualDriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSlowMode) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;
    this.isSlowModeFunc = isSlowMode;
    xLimiter = new SlewRateLimiter(4);
    yLimiter = new SlewRateLimiter(4);
    zLimiter = new SlewRateLimiter(4);
    addRequirements(m_swerveSubsystem); 
  }

  @Override
  public void execute() {
    // Get value
    // 負號加在這
    double xSpeed = -xSpeedFunc.getAsDouble();
    double ySpeed = -ySpeedFunc.getAsDouble();  
    double zSpeed = -zSpeedFunc.getAsDouble();
    boolean isSlowMode = isSlowModeFunc.getAsBoolean();
    // Dead band Limit
    Constants.DeadBandLimit(xSpeed, OperatorConstants.kJoystickDeadBand);
    Constants.DeadBandLimit(ySpeed, OperatorConstants.kJoystickDeadBand);
    Constants.DeadBandLimit(zSpeed, OperatorConstants.kJoystickDeadBand);
    // TurboModeSelect
    if(isSlowMode){
      xSpeed = xSpeed*0.4;
      ySpeed = ySpeed*0.4;
      zSpeed = zSpeed*0.4;
    }
    else{
      xSpeed = xSpeed*0.85;
      ySpeed = ySpeed*0.85;
      zSpeed = zSpeed*1;
    }
    // SlewRate
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    zSpeed = zLimiter.calculate(zSpeed);
    // Output
    m_swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
