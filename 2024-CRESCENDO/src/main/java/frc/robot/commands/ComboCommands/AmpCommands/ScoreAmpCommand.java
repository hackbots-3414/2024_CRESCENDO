// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ScoreAmpCommand extends Command {
  private Transport transport;
  private Shooter shooter;
  /** Creates a new ScoreAmpCommand. */
  public ScoreAmpCommand(Shooter shooter, Transport transport) {
    addRequirements(transport, shooter);
    this.transport = transport;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport.eject();
    shooter.setMotor(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.setNoteOnBoard(false);
    transport.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // this command should end by a timeout.
  }
}
