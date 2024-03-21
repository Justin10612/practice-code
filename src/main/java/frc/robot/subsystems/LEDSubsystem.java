// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    stopLED();
  }
  /* =========
   *   Blink
   * =========*/
  public void redBlink(){
    ledAnimation = new StrobeAnimation(255, 0, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void orangeBlink(){
    ledAnimation = new StrobeAnimation(255, 255, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void yellowBlink(){
    ledAnimation = new StrobeAnimation(255, 100, 0, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void purpleBlink(){
    ledAnimation = new StrobeAnimation(255, 0, 255, 0, 0.4, ledNum);
    m_candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  /* =========
   *   Solid
   * =========*/
  public void redSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(255, 0, 0);
    LEDConstants.LEDFlag = false;
  }
  public void greenSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(0, 255, 0);
    LEDConstants.LEDFlag = false;
  }
  public void blueSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(0, 0, 255);
    LEDConstants.LEDFlag = false;
  }
  public void purpleSolid(){
    m_candle.animate(null);
    m_candle.setLEDs(255, 0, 255);
    LEDConstants.LEDFlag = false;
  }
  public void stopLED(){
    m_candle.animate(null);
    m_candle.setLEDs(0, 0, 0);
    LEDConstants.LEDFlag = false;
  }

  @Override
  public void periodic() {
    // setRGB(LEDConstants.kHaveNoteRGBValue);
    // This method will be called once per scheduler run
    if(LEDConstants.LEDFlag){
      if(LEDConstants.playing == false) stopLED();
      else if(LEDConstants.aimReadyAMP && LEDConstants.speedReadyAMP && LEDConstants.haveApriltag) blueSolid();
      else if(LEDConstants.speedReadySPEAKER) purpleSolid();
      else if(LEDConstants.prepSPEAKER) purpleBlink();
      else if(LEDConstants.aimingAMP || LEDConstants.speedReadyAMP && LEDConstants.haveApriltag) orangeBlink();
      else if(LEDConstants.aimingAMP || LEDConstants.prepAMP && LEDConstants.haveApriltag == false) yellowBlink();
      else if(LEDConstants.hasNoteInSight && LEDConstants.trackingNote) orangeBlink();
      else if(LEDConstants.hasNoteInSight == false && LEDConstants.trackingNote) redBlink();
      else if(LEDConstants.intaking) redBlink();
      else if(LEDConstants.hasNote) greenSolid();
      else if(LEDConstants.hasNote == false) redSolid();
    }
    SmartDashboard.putBoolean("change", LEDConstants.aimingAMP);
  }
}
