// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command previouslyStoredCommand;
  private String previousCommandIdentifier;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Command newMethodCall = m_robotContainer.checkForOverrides();
    String newCommandIdentifier = m_robotContainer.currentOverride;

    // STEP 1: Check if already scheduled
    if (newMethodCall != null) {
      if (previouslyStoredCommand.isScheduled()) {
        if (newCommandIdentifier.equals(previousCommandIdentifier)) {
          previouslyStoredCommand.end(true);
          previouslyStoredCommand = newMethodCall;
          previouslyStoredCommand.schedule();
          previousCommandIdentifier = newCommandIdentifier; 
        }
      } else {
        previouslyStoredCommand = newMethodCall;
        previouslyStoredCommand.schedule();
        previousCommandIdentifier = newCommandIdentifier;
      }
    } else {
      if (previouslyStoredCommand.isScheduled()) { // might through null pointer
        previouslyStoredCommand.end(true);
        previousCommandIdentifier = null;
      }
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    Command latestCommand = m_robotContainer.checkForOverrides();
    String newCommandIdentifier = m_robotContainer.currentOverride;

    if (latestCommand != null) {
      if (previouslyStoredCommand.isScheduled()) {
          if (previouslyStoredCommand == null) previouslyStoredCommand.end(true);
          if (!previousCommandIdentifier.equals(newCommandIdentifier)) {
            latestCommand.schedule();
            previouslyStoredCommand = latestCommand;
            previousCommandIdentifier = newCommandIdentifier;
          }
      }
    }
  }
}
