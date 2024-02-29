// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimNoteCommand extends Command {
  /** Creates a new AimNoteCommand. */
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  // Inputs
  private DoubleSupplier xSpeedFunc;
  private BooleanSupplier isSlowModeFunc;
  // Slew rate limiter
  private final SlewRateLimiter xLimiter;
  public AimNoteCommand(LimeLightSubsystem limeLightSubsystem, SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, BooleanSupplier isSlowMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limeLightSubsystem = limeLightSubsystem;
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeed;
    this.isSlowModeFunc = isSlowMode;
    xLimiter = new SlewRateLimiter(4);
    addRequirements(m_limeLightSubsystem, m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Get value
    // 負號加在這
    double xSpeed = -xSpeedFunc.getAsDouble();
    double zSpeed = m_limeLightSubsystem.getTurnPIDOutput();
    boolean isSlowMode = isSlowModeFunc.getAsBoolean();
    // Dead band Limit
    Constants.DeadBandLimit(xSpeed, OperatorConstants.kJoystickDeadBand);
    // TurboModeSelect
    if(isSlowMode){
      xSpeed = xSpeed*0.4;
      zSpeed = zSpeed*1;
    }
    else{
      xSpeed = xSpeed*0.85;
      zSpeed = zSpeed*1;
    }
    // SlewRate
    xSpeed = xLimiter.calculate(xSpeed);
    // Output
    m_swerveSubsystem.drive(xSpeed, 0, zSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
