// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.EjectNoteIdelPose;
import frc.robot.commands.EjectNoteIntakePose;
// import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.NoteBackCommand;
import frc.robot.commands.NoteShootInverseCommand;
import frc.robot.commands.ShooterMotorStopCommand;
import frc.robot.commands.ShooterPreparingForAMPCommand;
import frc.robot.commands.NoteShootCommand;
import frc.robot.commands.ShooterPreparingForSpeakerCommand;
import frc.robot.subsystems.ClimbSubsystem;
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
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  // private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser;

  public static final CommandXboxController armJoystick = new CommandXboxController(1);
  public static final CommandXboxController baseJoystick = new CommandXboxController(0);
  // private final CommandJoystick shooterJoystick = new CommandJoystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    m_swerveSubsystem.setDefaultCommand(new ManualDriveCommand(m_swerveSubsystem));
    m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem));

    configureBindings();

    NamedCommands.registerCommand("ShooterTurn", new ShooterPreparingForSpeakerCommand(m_shooterSubsystem).beforeStarting(new ShooterPreparingForSpeakerCommand(m_shooterSubsystem)));

    NamedCommands.registerCommand("NoteIn", new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem).withTimeout(2));

    NamedCommands.registerCommand("NoteShoot", new NoteShootCommand(m_shooterSubsystem).withTimeout(0.5));

    NamedCommands.registerCommand("BaseStop", Commands.runOnce(()->{
      m_swerveSubsystem.drive_auto(new ChassisSpeeds(0, 0, 0));
    }));

    
  }

  private void configureBindings() {
    baseJoystick.b().onTrue(Commands.runOnce(()->{
      m_swerveSubsystem.resetGyro();
    }));

    armJoystick.x().whileTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem));
    armJoystick.rightBumper().whileTrue(new NoteShootCommand(m_shooterSubsystem));
    armJoystick.rightTrigger(0.4).whileTrue(new ShooterPreparingForSpeakerCommand(m_shooterSubsystem));
    armJoystick.leftTrigger(0.4).whileTrue(new ShooterPreparingForAMPCommand(m_shooterSubsystem));
    armJoystick.a().whileTrue(new EjectNoteIntakePose(m_intakeSubsystem, m_shooterSubsystem));
    armJoystick.b().whileTrue(new EjectNoteIdelPose(m_intakeSubsystem, m_shooterSubsystem));
    armJoystick.y().whileTrue(new NoteBackCommand(m_shooterSubsystem));
    armJoystick.pov(0).whileTrue(new NoteShootInverseCommand(m_shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
