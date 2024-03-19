// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle m_candle;
  private final CANdleConfiguration candleConfig;
  private final int ledNum = LEDConstants.kLedNum;
  private Animation ledAnimation = null;

  public LEDSubsystem() {
    m_candle = new CANdle(LEDConstants.kCANdleID);
    candleConfig = new CANdleConfiguration();
    candleConfig.stripType = LEDStripType.RGB;
    candleConfig.statusLedOffWhenActive = true;
    candleConfig.disableWhenLOS = false;
    candleConfig.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(candleConfig);
    //
    int[] off = {0, 0, 0};
    setRGB(off);
  }
  /* =========
   *   Blink
   * =========*/
  public void redBlink(){
    ledAnimation = new StrobeAnimation(255, 0, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
  }
  public void greenBlink(){
    ledAnimation = new StrobeAnimation(0, 255, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
  }
  public void BlueBlink(){
    ledAnimation = new StrobeAnimation(0, 0, 255, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
  }
  /* =========
   *   Solid
   * =========*/
  public void redSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(255, 0, 0);
  }
  public void greenSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(0, 255, 0);
  }
  public void blueSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(0, 0, 255);
  }

  public void setRGB(int rgb[]){
    ledAnimation = new StrobeAnimation(rgb[0], rgb[1], rgb[2], 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
  }

  public void clearLED(){
    ledAnimation = new StrobeAnimation(0, 0, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
  }

  @Override
  public void periodic() {
    // setRGB(LEDConstants.kHaveNoteRGBValue);
    // This method will be called once per scheduler run
  }
}
