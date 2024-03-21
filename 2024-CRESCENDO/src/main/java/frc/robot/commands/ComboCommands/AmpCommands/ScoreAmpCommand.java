// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ScoreAmpCommand extends Command {
  private Transport transport;
  private Shooter shooter;
  private Elevator elevator;
  public ScoreAmpCommand(Shooter shooter, Transport transport, Elevator elevator) {
    addRequirements(transport, shooter);
    this.transport = transport;
    this.shooter = shooter;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    if (elevator.isAtSetpoint()) {
      transport.eject();
      shooter.setMotor(-0.2);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}