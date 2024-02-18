// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;

public class LedSubsystem extends SubsystemBase {
  // final static Logger logger = LoggerFactory.getLogger(LedSubsystem.class);
  private static final double DEFAULT = 0.91;
  private static final double IN_RANGE = 0.83;
  private static final double END_GAME_0 = -0.87;
  private static final double END_GAME_15 = -0.11;
  private static final double END_GAME_30 = -0.21;
  private static final double NOTE_IN_VIEW = 0.65;
  private static final double CLIMBER_ACTIVATED = -0.95;

  Spark ledcontroller;

  public LedSubsystem() {
    ledcontroller = new Spark(0);
    ledcontroller.set(DEFAULT);
  }

  @Override
  public void periodic() {
    super.periodic();
    SubsystemManager subsystemManager = SubsystemManager.getInstance();
    if (subsystemManager.transport.getIR() && subsystemManager.drivetrain.isInRange()) {
      setColor(IN_RANGE);
    } else if (subsystemManager.transport.getIR()) {
      setColor(IN_RANGE);
    } else if (DriverStation.getMatchTime() < 0.1) {
      setColor(END_GAME_0);
    } else if (DriverStation.getMatchTime() < 15) {
      setColor(END_GAME_15);
    } else if (DriverStation.getMatchTime() < 30) {
      setColor(END_GAME_30);
    } else if (subsystemManager.noteFinder.isNoteDetected()) {
      setColor(NOTE_IN_VIEW);
    } else if (false) {
      // TODO make Climber subsystem
      setColor(CLIMBER_ACTIVATED);
    } else {
      setColor(DEFAULT);
    }

  }

  public void setColor(double color) {
    ledcontroller.set(color);
  }
}