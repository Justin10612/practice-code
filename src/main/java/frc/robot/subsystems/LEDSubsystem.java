// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CANdleConfiguration candleConfig;
  public LEDSubsystem() {
    candle = new CANdle(45);
    candleConfig = new CANdleConfiguration();

    candleConfig.vBatOutputMode = VBatOutputMode.On;
    candleConfig.stripType = LEDStripType.RGB;
    candle.configAllSettings(candleConfig);
    candle.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
