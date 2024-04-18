package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class ShooterCommand extends Command {
  Shooter shooter;
  Transport transport;
  boolean alreadyRanFeed;
  boolean alreadyRanShooter;
  double ticks = 0;
  double speed;
  private ShooterPivot pivot = null;

  public ShooterCommand(Shooter shooter, Transport transport) {
    this(shooter, transport, null);
  }

  public ShooterCommand(Shooter shooter, Transport transport, ShooterPivot pivot) {
    this(shooter, transport, pivot, Constants.ShooterConstants.maxSpeed);
  }

  public ShooterCommand(Shooter shooter, Transport transport, double speed) {
    this(shooter, transport, null, speed);
  }

  public ShooterCommand(Shooter shooter, Transport transport, ShooterPivot pivot, double speed) {
    this.shooter = shooter;
    this.transport = transport;
    this.pivot = pivot;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    alreadyRanFeed = false;
    alreadyRanShooter = false;
    ticks = 0;
  }

  @Override
  public void execute() {
    if(!alreadyRanShooter){
      shooter.setVelocity(speed);
      alreadyRanShooter = true;
    }

    if(shooter.shooterAtSpeed() && !alreadyRanFeed){
      if (pivot != null && !pivot.isAtSetpoint()) {
        return;
      }
      transport.setFast();
      alreadyRanFeed = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (transport.getNoteOnBoard()) {
      ticks = 0;
    } else {
      ticks++;
    }

    return ticks > 15;
  }
}
