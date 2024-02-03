// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.NoteFinderConstants;
import frc.robot.RobotContainer.RepathChoices;

public class Robot extends TimedRobot {
  private boolean runSysID = false;

  private Command m_autonomousCommand;
  private Command previouslyStoredCommand = new InstantCommand(() -> System.out.println("non null command, never will actually do this pls hopefully"));
  private RepathChoices previousCommandIdentifier = RepathChoices.NULL;

  private RobotContainer m_robotContainer;
  private SysIdRoutineBot m_SysIdRoutineBot;

  @Override
  public void robotInit() {
    if (runSysID) {
      m_SysIdRoutineBot = new SysIdRoutineBot();
      m_SysIdRoutineBot.configureBindings();
    } else {
      m_robotContainer = new RobotContainer();
      addPeriodic(m_robotContainer.getNoteFinder()::dataReceiver, NoteFinderConstants.CYCLE_TIME, 0);
    }
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
    m_autonomousCommand = !runSysID ? m_robotContainer.getAutonomousCommand() : m_SysIdRoutineBot.getAutonomousCommand();

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
    if (!runSysID) {
      RepathChoices newCommandIdentifier = m_robotContainer.checkForOverrides();
      if (newCommandIdentifier != null) {
        if (!previousCommandIdentifier.equals(newCommandIdentifier)) {
          previouslyStoredCommand.cancel();
          previouslyStoredCommand = m_robotContainer.getRepathingCommand(newCommandIdentifier);
          previouslyStoredCommand.schedule();
          previousCommandIdentifier = newCommandIdentifier;
        } else {
          if (m_robotContainer.isAtSetpoint(previousCommandIdentifier)) {
            previouslyStoredCommand.cancel();
          }
        }
      } else {
        if (previouslyStoredCommand.isScheduled()) {previouslyStoredCommand.cancel();}
        previousCommandIdentifier = RepathChoices.NULL;
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
  public void simulationPeriodic() {}
}
