// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_overrideCommand;
  private String m_specificOverrideCommand;

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
    if (m_overrideCommand != null) {
      if (m_overrideCommand.isScheduled()) { // is already scheduled
        // check if you dont wanna override anymore
        if (m_robotContainer.checkForOverrides() == null) { // if overrides are null
          m_overrideCommand.end(true); // cancel command
          m_specificOverrideCommand = null; // reset repeat command identifier
        } else { // if override is not null
          // check if the override isnt equal to previous override
          if (newCommandIdentifier.equals(m_specificOverrideCommand)) { // overrides aren't equal
            m_overrideCommand.end(true); // end old command
            m_overrideCommand = newMethodCall; // save new command
            m_overrideCommand.schedule(); // run new command
            m_specificOverrideCommand = newCommandIdentifier; // save command identifier
          } else {} // current button pressed is the same as the previous so do nothing :)
        }
      } else { // not yet scheduled
        // check if you wanna schedule
        if (m_overrideCommand != null) { // want to schedule new command
          m_overrideCommand = m_robotContainer.checkForOverrides();
          m_overrideCommand.schedule();
          m_specificOverrideCommand = newCommandIdentifier;
        } else {} // aren't scheduled yet, and dont wanna be scheduled
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
    Command newMethodCall = m_robotContainer.checkForOverrides();
    String newCommandIdentifier = m_robotContainer.currentOverride;

    // STEP 1: Check if already scheduled
    if (m_overrideCommand != null) {
      if (m_overrideCommand.isScheduled()) { // is already scheduled
        // check if you dont wanna override anymore
        if (m_robotContainer.checkForOverrides() == null) { // if overrides are null
          m_overrideCommand.end(true); // cancel command
          m_specificOverrideCommand = null; // reset repeat command identifier
        } else { // if override is not null
          // check if the override isnt equal to previous override
          if (newCommandIdentifier.equals(m_specificOverrideCommand)) { // overrides aren't equal
            m_overrideCommand.end(true); // end old command
            m_overrideCommand = newMethodCall; // save new command
            m_overrideCommand.schedule(); // run new command
            m_specificOverrideCommand = newCommandIdentifier; // save command identifier
          } else {} // current button pressed is the same as the previous so do nothing :)
        }
      } else { // not yet scheduled
        // check if you wanna schedule
        if (m_overrideCommand != null) { // want to schedule new command
          m_overrideCommand = m_robotContainer.checkForOverrides();
          m_overrideCommand.schedule();
          m_specificOverrideCommand = newCommandIdentifier;
        } else {} // aren't scheduled yet, and dont wanna be scheduled
      }
    }
  }
}
