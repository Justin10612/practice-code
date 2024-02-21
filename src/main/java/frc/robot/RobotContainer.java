// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeBackCommand;
// import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDriveCommand;
// import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController armJoystick = new CommandXboxController(1);
  public static final CommandXboxController baseJoystick = new CommandXboxController(0);
  // private final CommandJoystick shooterJoystick = new CommandJoystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new ManualDriveCommand(m_swerveSubsystem));
    // m_shooterSubsystem.setDefaultCommand(new IntakeBackCommand(m_intakeSubsystem, m_shooterSubsystem));
    // m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    baseJoystick.b().onTrue(Commands.runOnce(()->{
      m_swerveSubsystem.resetGyro();
    }));

    armJoystick.rightTrigger(0.4).whileTrue(Commands.run(()->{
      m_shooterSubsystem.shooterMotorTurn();
    }, m_shooterSubsystem));

    armJoystick.rightTrigger(0.4).onFalse(Commands.run(()->{
      m_shooterSubsystem.shooterMotorstop();
    }, m_shooterSubsystem));


    // shooterJoystick.button(6).whileTrue(Commands.run(()->{
    //   m_shooterSubsystem.shaftTurn(shooterJoystick.getRawAxis(1)*0.2);
    // }, m_shooterSubsystem));

    // shooterJoystick.button(6).whileFalse(Commands.runOnce(()->{
    //   m_shooterSubsystem.shaftStop();
    // }, m_shooterSubsystem));

    armJoystick.x().whileTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem, IntakeConstants.intakeInPosition, true, true,true));
    // armJoystick.b().onTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem, IntakeConstants.intakeInPosition, true, true,false));
    armJoystick.a().onTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem, IntakeConstants.intakePrimetivePosition, false, false,true));
    // armJoystick.y().onTrue(new ShooterCommand(m_shooterSubsystem, ShooterConstants.shooterAMPSetpoint));
    // armJoystick.b().onTrue(new ShooterCommand(m_shooterSubsystem, ShooterConstants.shooterPrimetivePosition));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
