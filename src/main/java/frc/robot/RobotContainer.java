// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbManually;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.EjectNoteIdlePose;
import frc.robot.commands.EjectNoteIntakePose;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ShooterEjectNote;
import frc.robot.commands.ShooterPrepAMP;
import frc.robot.commands.ShootAMP;
import frc.robot.commands.ShootSPEAKER;
import frc.robot.commands.ShooterPrepSPEAKER;
import frc.robot.commands.StopBase;
import frc.robot.commands.TrackNote_LimeLight;
import frc.robot.commands.TrackNote_PhotonVision;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final PhotonVisionSubsystem m_photonvisionSubsystem = new PhotonVisionSubsystem();
  // private final NoteDetectionSubsystem m_photonvisionSubsystem = new NoteDetectionSubsystem();
  private final LimeLightSubsystem m_LimeLightSubsystem = new LimeLightSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController DriverJoystick = new CommandXboxController(OperatorConstants.kDriverJoystickPort);
  private final CommandXboxController OperatorJoystick = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    NamedCommands.registerCommand("ClimbBack", new ClimbDown(m_ClimbSubsystem).withTimeout(0.5));

    // NamedCommands.registerCommand("TrackNote", new TrackNote(m_SwerveSubsystem, m).withTimeout(3));

    NamedCommands.registerCommand("ShooterTurnSPEAKER", new ShooterPrepSPEAKER(m_ShooterSubsystem).withTimeout(0.2));

    NamedCommands.registerCommand("ShooterTurnAMP", new ShooterPrepAMP(m_ShooterSubsystem).withTimeout(0.2));

    NamedCommands.registerCommand("LeftNoteInLeftSide", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(3));

    NamedCommands.registerCommand("LeftNoteInCenter", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(7));

    NamedCommands.registerCommand("RightNoteInCenter", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(6));
    
    NamedCommands.registerCommand("RightNoteInRightSide", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(4));

    NamedCommands.registerCommand("CenterNoteInCenter", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(4));

    NamedCommands.registerCommand("CenterLeftFirstNoteInCenter", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(5));

    NamedCommands.registerCommand("CenterCenterNoteInCenter", new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem).withTimeout(7));
    
    BooleanSupplier feedBtn = () -> OperatorJoystick.rightBumper().getAsBoolean();
    NamedCommands.registerCommand("NoteShootSPEAKER", new ShootSPEAKER(
      m_ShooterSubsystem,
      m_IndexerSubsystem,
      feedBtn,
      true).withTimeout(0.5));

    NamedCommands.registerCommand("NoteShootSPEAKERForEnd", new ShootSPEAKER(
      m_ShooterSubsystem,
      m_IndexerSubsystem,
      feedBtn,
      true));

    NamedCommands.registerCommand("NoteShootAMP", new ShootAMP(
      m_ShooterSubsystem,
      m_IndexerSubsystem,
      feedBtn,
      true).withTimeout(1));

    NamedCommands.registerCommand("ClimbUp", new ClimberUp(m_ClimbSubsystem).withTimeout(0.5));
    /* Create Chooser */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);

    configureBindings();

    // SmartDashboard.putData("Joystick/Driver", DriverJoystick.);
    // SmartDashboard.putData("Joysticl/Operator", OperatorJoystick);
  }

  private void configureBindings() {
    /* ============
     *    Driver 
     * ============
    */
    /* Manual Drive */
    DoubleSupplier xSpeedFunc = () -> DriverJoystick.getRawAxis(1);
    DoubleSupplier ySpeedFunc = () -> DriverJoystick.getRawAxis(0);
    DoubleSupplier zSppedFunc = () -> DriverJoystick.getRawAxis(4);
    BooleanSupplier isSlowModeFunc = () -> DriverJoystick.getHID().getRightTriggerAxis()>0.4;
    m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSppedFunc, isSlowModeFunc));
    /* Reset Gyro */
    DriverJoystick.b().whileTrue(
      Commands.runOnce(()->{
        m_SwerveSubsystem.resetGyro();
    }));
    /* Stop Base */
    DriverJoystick.x().whileTrue(new StopBase(m_SwerveSubsystem));
    /* Aiming Note */
    DriverJoystick.leftTrigger(0.4).and(OperatorJoystick.x()).whileTrue(new TrackNote_LimeLight(m_SwerveSubsystem, m_LimeLightSubsystem));
    /* Aiming Amp */
    // DriverJoystick.rightTrigger(0.4).whileTrue(new AimAMP(m_photonvisionSubsystem, m_SwerveSubsystem)); 

    /* ===========
     *   Operator 
     * ===========
    */
    BooleanSupplier feedBtn = () -> OperatorJoystick.getHID().getRightBumper();
    /* Climb */
    DoubleSupplier lInputFunc = () -> OperatorJoystick.getLeftY();
    DoubleSupplier rInputFunc = () -> OperatorJoystick.getRightY();
    BooleanSupplier enableFunc = () -> OperatorJoystick.getHID().getLeftBumper();
    m_ClimbSubsystem.setDefaultCommand(new ClimbManually(m_ClimbSubsystem, lInputFunc, rInputFunc, enableFunc));
    /* Intake Note */
    OperatorJoystick.x().whileTrue(
      new IntakeCommand(m_IntakeSubsystem, m_IndexerSubsystem));
    /* Eject Note when Intake is at down position. */
    OperatorJoystick.a().whileTrue(new EjectNoteIntakePose(m_IntakeSubsystem, m_IndexerSubsystem));
    /* Eject Note when Intake is at idle position. */
    OperatorJoystick.b().whileTrue(new EjectNoteIdlePose(m_IntakeSubsystem, m_IndexerSubsystem));
    /* Move Note backward */
    OperatorJoystick.y().whileTrue(new ShooterEjectNote(m_IndexerSubsystem));
    /* Spin Shooter for Speaker */
    OperatorJoystick.rightTrigger(0.4).whileTrue(
      new ShootSPEAKER(
        m_ShooterSubsystem,
        m_IndexerSubsystem,
        feedBtn,
        false));
    /* Spin Shooter for AMP */
    OperatorJoystick.leftTrigger(0.4).whileTrue(
      new ShootAMP(
        m_ShooterSubsystem,
        m_IndexerSubsystem,
        feedBtn,
        false));
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
