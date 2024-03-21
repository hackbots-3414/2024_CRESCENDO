package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransportConstants;
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
      transport.setEject();
      Timer.delay(TransportConstants.transportEjectDelay);
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