// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoScoreCommands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class PivotWait extends Command {
  private Logger logger = LoggerFactory.getLogger(PivotWait.class);
  private ShooterPivot pivot;
  private Transport transport;
  /** Creates a new PivotWait. */
  public PivotWait(ShooterPivot pivot, Transport transport) {
    this.pivot = pivot;
    this.transport = transport;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.debug("PivotWait started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.debug("PivotWait ended");
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.isAtSetpoint() || !transport.getNoteOnBoard(); // if we failed to intake, then we don't feel like waiting.
  }
}
