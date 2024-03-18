// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemManager;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends LoggedRobot {
  private boolean runSysID = false;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SysIdRoutineBot m_SysIdRoutineBot;

  @Override
  public void robotInit() {
   // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    // if (isReal()) {
    //   Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //   Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //   new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // } //else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    if (runSysID) {
      m_SysIdRoutineBot = new SysIdRoutineBot();
      m_SysIdRoutineBot.configureBindings();
    } else {
      m_robotContainer = RobotContainer.getInstance();
      // CameraServer.startAutomaticCapture();
      // addPeriodic(m_robotContainer.getNoteFinder()::dataReceiver,
      // NoteFinderConstants.CYCLE_TIME, 0);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = !runSysID ? m_robotContainer.getAutonomousCommand()
        : m_SysIdRoutineBot.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
    SubsystemManager.getInstance().resetAfterAuton().schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
