// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RevShooterMaxCommand extends Command {
  private Shooter shooter;
  private boolean setShooter;
  
  public RevShooterMaxCommand(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    setShooter = false;
  }

  @Override
  public void execute() {
    if (!setShooter) {
      shooter.setMaxSpeed();
      setShooter = true;
    }
  }
}
