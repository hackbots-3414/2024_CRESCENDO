// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.swing.GroupLayout.Alignment;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

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
  private static double matchTime = 0;

  // private static final double NOTE_IN_VIEW = 0.65;
  /// private static final double CLIMBER_ACTIVATED = -0.95;
  // private static final double DEFAULT = 0.91; // PURPPLE
  private static final FireAnimation HACKBOTS_FIRE = new FireAnimation(1.0, 0.1, LEDConstants.numLED, 0.2, 0.2);
  private static final StrobeAnimation HACKBOTS_STROBE = new StrobeAnimation(255, 255, 255, 0,
      LEDConstants.strobeSpeed,
      LEDConstants.numLED);
  private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_LEFT = new SingleFadeAnimation(0, 255, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_RIGHT = new SingleFadeAnimation(0, 255, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.rightNumLED, LEDConstants.rightOffset);
  private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_TOP = new SingleFadeAnimation(0, 255, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.topNumLED, LEDConstants.topOffset);
  private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_LEFT = new SingleFadeAnimation(250, 120, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_RIGHT = new SingleFadeAnimation(250, 120, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.rightNumLED, LEDConstants.rightOffset);
  private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_TOP = new SingleFadeAnimation(250, 120, 0, 0,
      LEDConstants.flashSpeed, LEDConstants.topNumLED, LEDConstants.topOffset);
  private static final StrobeAnimation RED_STROBE_ANIMATION_LEFT = new StrobeAnimation(255, 0, 0, 0,
      LEDConstants.strobeSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  private static final StrobeAnimation RED_STROBE_ANIMATION_RIGHT = new StrobeAnimation(255, 0, 0, 0,
      LEDConstants.strobeSpeed, LEDConstants.rightNumLED, LEDConstants.rightOffset);
  private static final TwinkleAnimation TWINKLE_ANIMATION = new TwinkleAnimation(255, 0, 255, 0, 0.5,
      LEDConstants.numLED, TwinklePercent.Percent42);
  private static final LarsonAnimation LARSON_ANIMATION = new LarsonAnimation(255, 0, 255, 0, 0.1, LEDConstants.numLED,
      LarsonAnimation.BounceMode.Back, 1);
  private Supplier<Boolean> noteOnBoard, isInRange, noteInView;
  private boolean noteOnBoardTest = false;
  private boolean isInRangeTest = false;
  private boolean noteInViewTest = false;
  // private int r = 0;
  // private int g = 0;
  // private int b = 0;
  private static final String[] LABELS = { "In Range", "Note Onboard", "End Game 15", "End Game 30", "Target Locked" };

  private static enum LED_MODE {
    IN_RANGE, NOTE_ONBOARD, END_GAME_WARNING, END_GAME_ALERT, ALIGNED, DEFAULT, NOTE_IN_VIEW;
  };

  private static LED_MODE chosenMode = LED_MODE.DEFAULT;

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  private int currentMode = 0;

  public LedSubsystem(Supplier<Boolean> noteOnBoard, Supplier<Boolean> isInRange, Supplier<Boolean> noteInView) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
    ledcontroller.configAllSettings(config);
    ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.clearAnimation(2);
    ledcontroller.setLEDs(255, 0, 255);
    // ledcontroller.animate(LARSON_ANIMATION);
    // this.noteOnBoard = noteOnBoard;
    // this.isInRange = isInRange;
    // this.noteInView = noteInView;
    // Hackbot Purple Code : [0x67, 0x2C, 0x91]
    // #672C91
    SmartDashboard.putBoolean("noteOnBoardTest", noteOnBoardTest);
    SmartDashboard.putBoolean("IsInRangeTest", isInRangeTest);
    SmartDashboard.putBoolean("noteInViewTest", noteInViewTest);
    // SmartDashboard.putNumber("r", r);
    // SmartDashboard.putNumber("g", g);
    // SmartDashboard.putNumber("b", b);
  }

  private void resetStatus() {
    for (String element : LABELS) {
      SmartDashboard.putBoolean(element, false);
    }
  }

  @Override
  public void periodic() {

    noteOnBoardTest = SmartDashboard.getBoolean("noteOnBoardTest", noteOnBoardTest);
    isInRangeTest = SmartDashboard.getBoolean("IsInRangeTest", isInRangeTest);
    noteInViewTest = SmartDashboard.getBoolean("noteInViewTest", noteInViewTest);
    // r = (int)SmartDashboard.getNumber("r", r);
    // g = (int)SmartDashboard.getNumber("g", g);
    // b = (int)SmartDashboard.getNumber("b", b);
    // ledcontroller.setLEDs(r, g, b);
    matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Countdown", matchTime);
    resetStatus();

    // / Note In View: Yellow
    // Note on Board : Green Medium Flash (During END Game Make sure this Overides
    // the End Game Alert or Warning 3 sec timer)
    // End Game Warning (20): White
    // End Game Alert (10): Strobe White
    // Shoot Alignment Happening : Slow Blue
    // When Aligned: Stop Strobe: Fast Flash Blue
    // InRange for shooting: Blue
    if (matchTime > LEDConstants.endgameWarning) {
      if (noteOnBoardTest && isInRangeTest) {
        // noteOnboardTest should be noteOnBoard.get()
        // Do this for all test Variables
        if (chosenMode != LED_MODE.IN_RANGE) {
          chosenMode = LED_MODE.IN_RANGE;
          ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.clearAnimation(2);
          ledcontroller.setLEDs(0, 0, 255, 0, LEDConstants.leftOffset, LEDConstants.leftNumLED);
          ledcontroller.setLEDs(0, 0, 255, 0, LEDConstants.rightOffset, LEDConstants.rightNumLED);
          ledcontroller.setLEDs(0, 0, 255, 0, LEDConstants.topOffset, LEDConstants.topNumLED); // solid blue
        }
      } else if (noteOnBoardTest) {
        if (chosenMode != LED_MODE.NOTE_ONBOARD) {
          chosenMode = LED_MODE.NOTE_ONBOARD;
          ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.clearAnimation(2);
          ledcontroller.animate(GREEN_FLASH_ANIMATION_TOP, 0);
          ledcontroller.animate(GREEN_FLASH_ANIMATION_RIGHT, 1);
          ledcontroller.animate(GREEN_FLASH_ANIMATION_LEFT, 2); // flashing green
        }
      } else if (noteInViewTest) {
        if (chosenMode != LED_MODE.NOTE_IN_VIEW) {
          chosenMode = LED_MODE.NOTE_IN_VIEW;

          ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.clearAnimation(2);
          ledcontroller.animate(YELLOW_FLASH_ANIMATION_LEFT, 0);
          ledcontroller.animate(YELLOW_FLASH_ANIMATION_RIGHT, 1);
          ledcontroller.animate(YELLOW_FLASH_ANIMATION_TOP, 2);
        }
      }

      else {
        if (chosenMode != LED_MODE.DEFAULT) {
          chosenMode = LED_MODE.DEFAULT;
          ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.clearAnimation(2);
          ledcontroller.setLEDs(255, 0, 255, 0, LEDConstants.leftOffset, LEDConstants.leftNumLED);
          ledcontroller.setLEDs(255, 0, 255, 0, LEDConstants.rightOffset, LEDConstants.rightNumLED);
          ledcontroller.setLEDs(255, 0, 255, 0, LEDConstants.topOffset, LEDConstants.topNumLED);
        }

      }
    }
  }
  /*
   * if (matchTime <= LEDConstants.endgameWarning) {
   * if (chosenMode != LED_MODE.END_GAME_WARNING) {
   * chosenMode = LED_MODE.END_GAME_WARNING;
   * ledcontroller.clearAnimation(0);
   * ledcontroller.setLEDs(255, 0, 0);
   * else if (matchTime < LEDConstants.endgameAlert) {
   * if (chosenMode != LED_MODE.END_GAME_ALERT) {
   * chosenMode = LED_MODE.END_GAME_ALERT;
   * ledcontroller.clearAnimation(0);
   * ledcontroller.animate(WHITE_STROBE_ANIMATION);
   * // ledcontroller.animate();
   * else if (matchTime < LEDConstants.endgameWarning) {
   * if (chosenMode != LED_MODE.END_GAME_WARNING) {
   * chosenMode = LED_MODE.END_GAME_WARNING;
   * ledcontroller.clearAnimation(0);
   * ledcontroller.setLEDs(255, 0, 0); // red
   * }
   */

  public void setColor(double color) {
    // ledcontroller.set(color);
  }
}