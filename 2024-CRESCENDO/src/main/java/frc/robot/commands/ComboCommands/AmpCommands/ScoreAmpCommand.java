package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class ScoreAmpCommand extends Command {
  private Transport transport;
  private Shooter shooter;
  private Elevator elevator;

  private double goalTicks = 0.5 * 50;
  private double ticks = 0;
  
  public ScoreAmpCommand(Shooter shooter, Transport transport, Elevator elevator, ShooterPivot pivot) {
    addRequirements(transport, shooter, pivot);
    this.transport = transport;
    this.shooter = shooter;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    if (elevator.isAtSetpoint()) {
      transport.setEject();
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
    ticks++;
    return ticks > goalTicks;
  }
}