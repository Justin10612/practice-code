// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LEDConstants;

// public class LEDSubsystem extends SubsystemBase {
//   /** Creates a new LEDSubsystem. */
//   private final CANdle candle;
//   private final CANdleConfiguration candleConfig;
//   private final int ledNum = LEDConstants.kLedNum;

//   private Animation m_toAnimate = null;

//   public LEDSubsystem() {
//     candle = new CANdle(LEDConstants.kCANdleID);
//     candleConfig = new CANdleConfiguration();
//     candleConfig.stripType = LEDStripType.RGB;
//     candleConfig.statusLedOffWhenActive = true;
//     candleConfig.disableWhenLOS = false;
//     candleConfig.vBatOutputMode = VBatOutputMode.Modulated;
//     candle.configAllSettings(candleConfig);
//   }
//   public void setAnimation(Animation animation){
//     this.m_toAnimate = animation;
//   }
//   @Override
//   public void periodic() {
//     candle.animate(m_toAnimate);
//     // This method will be called once per scheduler run
//   }
// }
