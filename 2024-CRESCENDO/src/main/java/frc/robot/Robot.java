// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command previouslyStoredCommand = new InstantCommand(() -> System.out.println("non null command, never will actually do this pls hopefully"));
  private String previousCommandIdentifier = "nothing";

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
    if (newMethodCall != null) {
      if (!previousCommandIdentifier.equals(newCommandIdentifier)) {
        previouslyStoredCommand.cancel();
        previouslyStoredCommand = newMethodCall;
        previouslyStoredCommand.schedule();
        previousCommandIdentifier = newCommandIdentifier;
      }
    } else {
      if (previouslyStoredCommand.isScheduled()) {
        previouslyStoredCommand.cancel();
      }
      previousCommandIdentifier = "nothing";
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
  public void simulationPeriodic() {}
}
