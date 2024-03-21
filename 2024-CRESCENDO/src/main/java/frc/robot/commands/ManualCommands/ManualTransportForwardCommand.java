package frc.robot.commands.ManualCommands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;
public class ManualTransportForwardCommand extends Command {
  private Transport transport;
  
  public ManualTransportForwardCommand(Transport transport) {
    addRequirements(transport);
    this.transport = transport;
  }

  @Override
  public void initialize() {
    transport.setMotor(Constants.TransportConstants.transportEjectSpeed * -1);
  }

  @Override
  public void end(boolean interrupted) {
    transport.stopMotor();
  }
}
