// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

public class LedSubsystem extends SubsystemBase {
  // final static Logger logger = LoggerFactory.getLogger(LedSubsystem.class);
  private static final double IN_RANGE = 0.87; // BLUE
  private static final double NOTE_ONBOARD = 0.77; // GREEN
  private static final double END_GAME_15 = -0.61; // RED
  private static final double END_GAME_30 = -0.93; // WHITE
  private static final double AIM_READY = 0.69; // YELLOW
  // private static final double NOTE_IN_VIEW = 0.65;
  ///private static final double CLIMBER_ACTIVATED = -0.95;
  private static final double DEFAULT = 0.91; // PURPPLE

  private Supplier<Boolean> noteOnBoard, isInRange, aimReady;

  private static final String[] LABELS = {"In Range", "Note Onboard", "End Game 15", "End Game 30", "Target Locked"};

  Spark ledcontroller = new Spark(0);

  public LedSubsystem(Supplier<Boolean> noteOnBoard, Supplier<Boolean> isInRange, Supplier<Boolean> aimReady) {
    ledcontroller.set(DEFAULT);
    this.noteOnBoard = noteOnBoard;
    this.isInRange = isInRange;
    this.aimReady = aimReady;
  }

  private void resetStatus() {
    for(String element : LABELS) {
      SmartDashboard.putBoolean(element, false);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Countdown", DriverStation.getMatchTime());
    resetStatus();

    if (noteOnBoard.get() && isInRange.get()) {
      setColor(IN_RANGE);
      SmartDashboard.putBoolean(LABELS[0], true);
    } else if (noteOnBoard.get()) {
      setColor(NOTE_ONBOARD);
      SmartDashboard.putBoolean(LABELS[1], true);
    } else if (aimReady.get()) {
      setColor(AIM_READY);
      SmartDashboard.putBoolean(LABELS[4], true);
    } else if (DriverStation.getMatchTime() < 15) {
      setColor(END_GAME_15);
      SmartDashboard.putBoolean(LABELS[2], true);
      SmartDashboard.putBoolean(LABELS[3], true);
    } else if (DriverStation.getMatchTime() < 30) {
      setColor(END_GAME_30);
      SmartDashboard.putBoolean(LABELS[3], true);
    } else {
      setColor(DEFAULT);
    }
  }

  public void setColor(double color) {
    ledcontroller.set(color);
  }
}