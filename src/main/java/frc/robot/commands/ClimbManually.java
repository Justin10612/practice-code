// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimbManually extends Command {
  /** Creates a new ClimbCommand. */
  private final ClimbSubsystem m_climbSubsystem;
  private final DoubleSupplier m_leftFunc;
  private final DoubleSupplier m_rightFunc;
  private final BooleanSupplier m_enableBtnFunc;
  // Variable
  private double leftInputVal; 
  private double rightInputVal; 
  private boolean enable; 

  public ClimbManually(ClimbSubsystem climbSubsystem, DoubleSupplier leftFunc, DoubleSupplier rightFunc, BooleanSupplier enableBtn) {
    this.m_climbSubsystem = climbSubsystem;
    this.m_leftFunc= leftFunc;
    this.m_rightFunc = rightFunc;
    this.m_enableBtnFunc = enableBtn;
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Inputs
    leftInputVal = -m_leftFunc.getAsDouble();
    rightInputVal = -m_rightFunc.getAsDouble();
    enable = m_enableBtnFunc.getAsBoolean();
    // Output
    if(enable){
      m_climbSubsystem.setRightMotor(rightInputVal);
      m_climbSubsystem.setLeftMotor(leftInputVal);
    }else{
      m_climbSubsystem.StopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
