package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Transport;

public class TransportCommand extends Command {
  private Transport transport;
  private boolean forward;

  public TransportCommand(Transport transport, boolean forward) {
    this.transport = transport;
    addRequirements(transport);
  }

  @Override
  public void initialize() {
    transport.setMotor(forward ? Constants.TransportConstants.transportSpeed : Constants.TransportConstants.transportEjectSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    transport.setMotor(0.0);
  }
}
