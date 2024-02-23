// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

public class LedSubsystem extends SubsystemBase {
  // final static Logger logger = LoggerFactory.getLogger(LedSubsystem.class);
  private static final double IN_RANGE = 0.83;
  private static final double NOTE_ONBOARD = 0.77;
  private static final double END_GAME_0 = -0.87;
  private static final double END_GAME_15 = -0.11;
  private static final double END_GAME_30 = -0.21;
  private static final double NOTE_IN_VIEW = 0.65;
  ///private static final double CLIMBER_ACTIVATED = -0.95;
  private static final double DEFAULT = 0.91;

  private static final String[] LABELS = {
    "In Range", "Note Onboard", "End Game 0", "End Game 15", "End Game 30", "Note In View", "Climber Activated"
  };



  Spark ledcontroller = new Spark(0);

  public LedSubsystem() {
    ledcontroller.set(DEFAULT);
  }

  private void resetStatus() {
    for(String element : LABELS) {
      SmartDashboard.putBoolean(element, false);
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Countdown", DriverStation.getMatchTime());
    resetStatus();
    SubsystemManager subsystemManager = SubsystemManager.getInstance();
    if (subsystemManager.transport.getIR() && subsystemManager.drivetrain.isInRange()) {
      setColor(IN_RANGE);
      SmartDashboard.putBoolean(LABELS[0], true);

    } else if (subsystemManager.transport.getIR()) {
      setColor(NOTE_ONBOARD);
      SmartDashboard.putBoolean(LABELS[1], true);
      SmartDashboard.putBoolean(LABELS[0], true);

    } else if (DriverStation.getMatchTime() < 0.1) {
      setColor(END_GAME_0);
      SmartDashboard.putBoolean(LABELS[2], true);
      SmartDashboard.putBoolean(LABELS[3], true);
      SmartDashboard.putBoolean(LABELS[4], true);

    } else if (DriverStation.getMatchTime() < 15) {
      setColor(END_GAME_15);
      SmartDashboard.putBoolean(LABELS[3], true);
      SmartDashboard.putBoolean(LABELS[4], true);

    } else if (DriverStation.getMatchTime() < 30) {
      setColor(END_GAME_30);
      SmartDashboard.putBoolean(LABELS[4], true);

    } else if (subsystemManager.noteFinder.isNoteDetected()) {
      setColor(NOTE_IN_VIEW);
      SmartDashboard.putBoolean(LABELS[5], true);

    } /*else if (false) {
      // TODO make Climber subsystem
      setColor(CLIMBER_ACTIVATED);
      SmartDashboard.putBoolean(LABELS[6], true);
    } */else {
      setColor(DEFAULT);
    }

  }

  public void setColor(double color) {
    ledcontroller.set(color);
  }
}