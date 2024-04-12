// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualScheduler extends Command {
  private Command command;

  /**
   * A manual sceduler that will be run a command without using the actual sceduler. This is good.
   * @param commandSup a supplier that returns the command to scedule. Do NOT use a ProxyCommand
   */
  public ManualScheduler(Supplier<Command> commandSup) {
    command = commandSup.get();
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
