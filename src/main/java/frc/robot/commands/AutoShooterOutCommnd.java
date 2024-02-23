// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShooterOutCommnd extends Command {
  /** Creates a new AutoShooterOutCommnd. */
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final Timer time = new Timer();
  private double xspeed;
  private double robotPosition;

  private double nowTime;
  public AutoShooterOutCommnd(ShooterSubsystem _shooterSubsystem, SwerveSubsystem _swerveSubsystem) {
    this.shooterSubsystem = _shooterSubsystem;
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(shooterSubsystem, swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    swerveSubsystem.resetEncoder();
    swerveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    nowTime = time.get();
    robotPosition = swerveSubsystem.leftModulePosition();
    if (robotPosition <= 0.8 && nowTime <= 2) {
      shooterSubsystem.shooterMotorTurn(9.6);
      xspeed = 0.5;
    }
    else if(robotPosition >= 0 && nowTime <= 4){
      shooterSubsystem.shooterMotorTurn(9.6);
      xspeed = -0.5;
    }
    else if(nowTime <= 5){
      shooterSubsystem.shoot();
    }
    else{
      shooterSubsystem.shooterMotorstop();
      xspeed = 0;
    }
    swerveSubsystem.drive(xspeed, 0, 0, true);

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
