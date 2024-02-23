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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoShooterOutCommnd;
import frc.robot.commands.AutoTwoNoteCommand;
import frc.robot.commands.ClimbCommand;
// import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDriveCommand;
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
  private final AutoShooterOutCommnd m_autoShootrOutCommand = new AutoShooterOutCommnd(m_shooterSubsystem, m_swerveSubsystem);
  private final AutoTwoNoteCommand m_autoTwoNoteCommand = new AutoTwoNoteCommand(m_shooterSubsystem, m_intakeSubsystem, m_swerveSubsystem);

  // private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static final CommandXboxController armJoystick = new CommandXboxController(1);
  public static final CommandXboxController baseJoystick = new CommandXboxController(0);
  // private final CommandJoystick shooterJoystick = new CommandJoystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto mode", autoChooser);
    m_swerveSubsystem.setDefaultCommand(new ManualDriveCommand(m_swerveSubsystem));
    m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem));

    configureBindings();

    NamedCommands.registerCommand("ShooterTurn", Commands.runOnce(()->{
      m_shooterSubsystem.shooterMotorTurn(9.6);
    }));

    NamedCommands.registerCommand("NoteIn", Commands.runOnce(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakeInPosition);
      m_shooterSubsystem.shouldTransportTurn(true);
    }));

    NamedCommands.registerCommand("IntakeOut", Commands.runOnce(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakeInPosition);
      m_intakeSubsystem.shouldturn(true);
      m_shooterSubsystem.shouldTransportTurn(true);
    }));

    NamedCommands.registerCommand("IntakeBack", Commands.runOnce(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakePrimetivePosition);
      m_intakeSubsystem.shouldturn(false);
      m_shooterSubsystem.shouldTransportTurn(false);
    }));

    NamedCommands.registerCommand("AllStop", Commands.runOnce(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakePrimetivePosition);
      m_intakeSubsystem.shouldturn(false);
      m_shooterSubsystem.shooterMotorstop();
      m_shooterSubsystem.shouldTransportTurn(false);
    }));

    NamedCommands.registerCommand("BaseStop", Commands.runOnce(()->{
      m_swerveSubsystem.drive_auto(new ChassisSpeeds(0, 0, 0));
    }));

    
  }

  private void configureBindings() {
    baseJoystick.b().onTrue(Commands.runOnce(()->{
      m_swerveSubsystem.resetGyro();
    }));

    armJoystick.rightTrigger(0.4).whileTrue(Commands.run(()->{
      m_shooterSubsystem.shooterMotorTurn(ShooterConstants.shooterSpeakerSpeedSetpoint);;
    }, m_shooterSubsystem));

    armJoystick.rightTrigger(0.4).and(armJoystick.leftTrigger(0.4)).and(armJoystick.rightBumper()).whileFalse(Commands.runOnce(()->{
      m_shooterSubsystem.shooterMotorstop();
    }, m_shooterSubsystem));

    armJoystick.leftTrigger(0.4).whileTrue(Commands.run(()->{
      m_shooterSubsystem.shooterMotorTurn(ShooterConstants.shooterAMPSpeedSetpoint);;
    }, m_shooterSubsystem));

    armJoystick.b().whileTrue(Commands.run(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakeInPosition);
      m_intakeSubsystem.shouldturn(true);
      m_intakeSubsystem.turnReverse();
      m_shooterSubsystem.shouldTransportTurn(true);
    }, m_intakeSubsystem, m_shooterSubsystem));

    armJoystick.y().whileTrue(Commands.run(()->{
      m_intakeSubsystem.getintakeShaftSetpoint(IntakeConstants.intakePrimetivePosition);
      m_intakeSubsystem.shouldturn(true);
      m_intakeSubsystem.turnReverse();
      m_shooterSubsystem.shouldTransportTurn(true);
    }, m_intakeSubsystem, m_shooterSubsystem));

    armJoystick.leftBumper().whileTrue(Commands.run(()->{
      m_shooterSubsystem.shooterMotorTurn(ShooterConstants.shooterAMPSpeedSetpoint);
      m_shooterSubsystem.transportMotorTurn();
    }, m_shooterSubsystem));

    armJoystick.rightBumper().whileTrue(Commands.run(()->{
      m_shooterSubsystem.shoot();
    }, m_shooterSubsystem));

    autoChooser.setDefaultOption("none", null);
    autoChooser.addOption("ShooterOut", m_autoShootrOutCommand);
    autoChooser.addOption("TwoNoteAuto", m_autoTwoNoteCommand);
    SmartDashboard.putData(autoChooser);



    // shooterJoystick.button(6).whileTrue(Commands.run(()->{
    //   m_shooterSubsystem.shaftTurn(shooterJoystick.getRawAxis(1)*0.2);
    // }, m_shooterSubsystem));

    // shooterJoystick.button(6).whileFalse(Commands.runOnce(()->{
    //   m_shooterSubsystem.shaftStop();
    // }, m_shooterSubsystem));

    armJoystick.x().whileTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem, IntakeConstants.intakeInPosition, true, true));
    armJoystick.a().onTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem, IntakeConstants.intakePrimetivePosition, false, false));
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
    return autoChooser.getSelected();
  }
}
