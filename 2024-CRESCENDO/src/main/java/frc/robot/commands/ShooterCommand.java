package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterCommand extends Command {
  Shooter shooter;
  Transport transport;
  double velocity;

  public ShooterCommand(Shooter shooter, Transport transport, double velocity) {
    addRequirements(shooter, transport);
    this.shooter = shooter;
    this.transport = transport;
    this.velocity = velocity;
  }

  @Override
  public void initialize() {
    transport.setMotor(TransportConstants.transportEjectSpeed);
    shooter.setMotor(ShooterConstants.shooterIntakeSpeed);
  }

  @Override
  public void execute() {
    if (transport.getIR() == true) {
      transport.stopMotor();
      shooter.setVelocity(velocity);
    }
    if (shooter.shooterAtSpeed()) {
      transport.setMotor(TransportConstants.transportSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }
}
