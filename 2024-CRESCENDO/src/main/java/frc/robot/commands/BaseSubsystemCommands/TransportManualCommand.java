package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;
public class TransportManualCommand extends Command {
  private Transport transport;
  private int ticks;
  private int goalTicks;
  
  public TransportManualCommand(Transport transport, double seconds) {
    addRequirements(transport);
    this.transport = transport;
    this.goalTicks = (int) (seconds / 0.02);
  }

  @Override
  public void initialize() {
    transport.setFast();
  }

  @Override
  public boolean isFinished() {
      ticks++;
      return ticks >= goalTicks;
  }

  @Override
  public void end(boolean interrupted) {
    transport.stopMotor();
  }
}
