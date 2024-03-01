package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterCommand extends Command {
  Shooter shooter;
  Transport transport;
  double velocity;
  Consumer<Boolean> setNoteIsOnBoard;

  public ShooterCommand(Shooter shooter, Transport transport, double velocity, Consumer<Boolean> setNoteIsOnBoard) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.transport = transport;
    this.velocity = velocity;
    this.setNoteIsOnBoard = setNoteIsOnBoard;
  }

  @Override
  public void initialize() {
    transport.setMotor(TransportConstants.transportEjectSpeed);
    shooter.setMotor(ShooterConstants.shooterBackupSpeed);
  }

  @Override
  public void execute() {
    if (transport.getIR() == true) {
      transport.stopMotor();
      shooter.setVelocity(velocity);
    }
    if (shooter.shooterAtSpeed()) {
      transport.setMotor(TransportConstants.transportSpeed);
      setNoteIsOnBoard.accept(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }
}
