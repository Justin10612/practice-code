// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTwoNoteCommand extends Command {
  /** Creates a new AutoTwoNoteCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final Timer time = new Timer();
  private double nowTime;
  private double robotPosition;
  private double xspeed;
  public AutoTwoNoteCommand(ShooterSubsystem _shooterSubsystem, IntakeSubsystem _intakeSubsystem, SwerveSubsystem _swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = _shooterSubsystem;
    this.intakeSubsystem = _intakeSubsystem;
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    swerveSubsystem.resetEncoder();
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
    else if(robotPosition >= 0 && nowTime > 2 && nowTime <= 4){
      shooterSubsystem.shooterMotorTurn(9.6);
      xspeed = -0.5;
    }
    else if(nowTime > 4 && nowTime <= 5){
      shooterSubsystem.shoot();
      xspeed = 0;
    }
    else if(nowTime > 5 && nowTime <= 5.1){
      intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakeInPosition);
      intakeSubsystem.shouldturn(true);
      shooterSubsystem.shouldTransportTurn(true);
      xspeed = 0;
    }
    else if(robotPosition <= 1.35 && nowTime > 5.1 && nowTime <= 7.1){
      xspeed = 0.4;
    }
    else if(robotPosition >= 0 && nowTime > 7.1 && nowTime <= 9.1){
      xspeed = -0.5;
    }
    else if(nowTime > 9.1 && nowTime <= 12){
      xspeed = 0;
      shooterSubsystem.shoot();
    }
    else{
      shooterSubsystem.shooterMotorstop();
      intakeSubsystem.shouldturn(false);
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

