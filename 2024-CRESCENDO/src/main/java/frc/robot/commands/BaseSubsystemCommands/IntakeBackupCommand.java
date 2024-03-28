// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

/**
 * A command that will back up the note in the transport using the shooter and transport. This does not affect the intake motor, but is still called "intake" b/c it is part of the intake rutine
 */
public class IntakeBackupCommand extends Command {
  /** Creates a new IntakeBackupCommand. */
  private Transport transport;
  private Shooter shooter;
  public IntakeBackupCommand(Transport transport, Shooter shooter) {
    this.transport = transport;
    this.shooter = shooter;
    addRequirements(transport, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (transport.getTransportIR() && transport.getFlyWheelIR()) {
      transport.setBackup();
      shooter.setBackup();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transport.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !transport.getFlyWheelIR();
  }
}
