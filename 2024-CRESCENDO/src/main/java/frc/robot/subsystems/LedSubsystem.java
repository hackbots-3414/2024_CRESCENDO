// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.swing.GroupLayout.Alignment;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LEDConstants;

public class LedSubsystem extends SubsystemBase {
  // final static Logger logger = LoggerFactory.getLogger(LedSubsystem.class);
  private static final double IN_RANGE = 0.87; // BLUE
  private static final double NOTE_ONBOARD = 0.77; // GREEN
  private static final double END_GAME_15 = -0.61; // RED
  private static final double END_GAME_30 = -0.93; // WHITE
  private static final double AIM_READY = 0.69; // YELLOW
  private static double lastledchange = 0;
  private static double matchTime = 0;
  private static boolean animationDone = false;

  // private static final double NOTE_IN_VIEW = 0.65;
  /// private static final double CLIMBER_ACTIVATED = -0.95;
  // private static final double DEFAULT = 0.91; // PURPPLE
  private static final FireAnimation HACKBOTS_FIRE = new FireAnimation(1.0, 0.1, LEDConstants.numLED, 0.2, 0.2);
  private static final StrobeAnimation HACKBOTS_STROBE = new StrobeAnimation(255, 255, 255, 0,
      LEDConstants.strobeSpeed,
      LEDConstants.numLED);
  private static final SingleFadeAnimation GREEN_FLASH_ANIMATION = new SingleFadeAnimation(52, 143, 21, 0,
      LEDConstants.flashSpeed, LEDConstants.numLED);
  private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION = new SingleFadeAnimation(247, 255, 3, 0,
      LEDConstants.flashSpeed, LEDConstants.numLED);
  private static final StrobeAnimation WHITE_STROBE_ANIMATION = new StrobeAnimation(255, 255, 255, 0,
      LEDConstants.strobeSpeed, LEDConstants.numLED);

  private Supplier<Boolean> noteOnBoard, isInRange, noteInView;
  private boolean noteOnBoardTest = false; 
  private boolean isInRangeTest = false; 
  private boolean noteInViewTest = false;

  private static final String[] LABELS = { "In Range", "Note Onboard", "End Game 15", "End Game 30", "Target Locked" };

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  private int currentMode = 0;

  public LedSubsystem(Supplier<Boolean> noteOnBoard, Supplier<Boolean> isInRange, Supplier<Boolean> noteInView) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    // config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    ledcontroller.configAllSettings(config);
    ledcontroller.clearAnimation(0);
    // ledcontroller.animate(HACKBOTS_STROBE);
    // ledcontroller.setLEDs(0x67, 0x2C, 0x91);
    ledcontroller.clearAnimation(0);
    ledcontroller.animate(GREEN_FLASH_ANIMATION);
    this.noteOnBoard = noteOnBoard;
    this.isInRange = isInRange;
    this.noteInView = noteInView;
    // Hackbot Purple Code : [0x67,0x]
    // #672C91
    SmartDashboard.putBoolean("noteOnBoardTest", noteOnBoardTest);
    SmartDashboard.putBoolean("IsInRangeTest", isInRangeTest);
    SmartDashboard.putBoolean("noteInViewTest", noteInViewTest);
  }

  private void resetStatus() {
    for (String element : LABELS) {
      SmartDashboard.putBoolean(element, false);
    }
  }

  @Override
  public void periodic() {

    SmartDashboard.getBoolean("noteOnBoardTest", noteOnBoardTest);
    SmartDashboard.getBoolean("IsInRangeTest", isInRangeTest);
    SmartDashboard.getBoolean("noteInViewTest", noteInViewTest);

    matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Countdown", matchTime);
    resetStatus();

    animationDone = lastledchange - matchTime >= 3.0;


    // / Note In View: Yellow
    // Note on Board : Green Medium Flash (During END Game Make sure this Overides
    // the End Game Alert or Warning 3 sec timer)
    // End Game Warning (20): White
    // End Game Alert (10): Strobe White
    // Shoot Alignment Happening : Slow Blue
    // When Aligned: Stop Strobe: Fast Flash Blue
    // InRange for shooting: Blue

    if (noteOnBoard.get() && isInRange.get()) {
      ledcontroller.setLEDs(23, 2, 250); // solid blue
      lastledchange = matchTime;

    } else if (noteOnBoard.get()) {
      if (currentMode != 3) {
        currentMode = 3;
        ledcontroller.clearAnimation(0);
        ledcontroller.animate(GREEN_FLASH_ANIMATION); // flashing green
        lastledchange = matchTime;
      } else if (matchTime <= LEDConstants.endgameWarning) {
        if (currentMode != 1) {
          currentMode = 1;
          ledcontroller.clearAnimation(0);
          ledcontroller.setLEDs(255, 255, 255);
          lastledchange = matchTime;

        }
      }


    } else if (noteInView.get()) {
      if (currentMode != 4) {
        currentMode = 4;
        ledcontroller.clearAnimation(0);
        ledcontroller.animate(YELLOW_FLASH_ANIMATION);
        lastledchange = matchTime;

      }

    } else if (matchTime < LEDConstants.endgameAlert) {
      if (currentMode != 2) {
        currentMode = 2;
        ledcontroller.clearAnimation(0);
        ledcontroller.animate(WHITE_STROBE_ANIMATION);
        // ledcontroller.animate();
        lastledchange = matchTime;

      }
    } else if (matchTime < LEDConstants.endgameWarning) {
      if (currentMode != 1) {
        currentMode = 1;
        ledcontroller.clearAnimation(0);
        ledcontroller.setLEDs(255, 255, 255);
        lastledchange = matchTime;

      }
    } else {
      if (currentMode != 0) {
        currentMode = 0;
        ledcontroller.clearAnimation(0);
        ledcontroller.setLEDs(0x67, 0x2C, 0x91);
        // ledcontroller.animate(HACKBOTS_FIRE);
        ledcontroller.setLEDs(0x67, 0x2C, 0x91);
        lastledchange = matchTime;

      }
    }
  }

  public void setColor(double color) {
    // ledcontroller.set(color);
  }
}