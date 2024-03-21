package frc.robot.commands.BaseSubsystemCommands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterCommand extends Command {
  Shooter shooter;
  Transport transport;
  boolean seenNote;
  double ticks = 0;

  Optional<Supplier<Boolean>> waitToFeed;
  boolean feed = false;

  public ShooterCommand(Shooter shooter, Transport transport, Optional<Supplier<Boolean>> waitToFeed) {
    this.shooter = shooter;
    this.transport = transport;
    this.waitToFeed = waitToFeed;
  }

  @Override
  public void initialize() {
    seenNote = false;
    feed = false;
    ticks = 0;
  }

  @Override
  public void execute() {
    if (waitToFeed.isEmpty()) {
      feed = true;
    } else {
      feed = waitToFeed.get().get();
    }

    if (transport.getFlyWheelIR() && !seenNote) {
      // too far forward, back up
      shooter.setBackup();
      transport.setBackup();
      seenNote = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // shooter.setWarmUpSpeed();
    shooter.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return (seenNote && !transport.getFlyWheelIR());
  }
}
