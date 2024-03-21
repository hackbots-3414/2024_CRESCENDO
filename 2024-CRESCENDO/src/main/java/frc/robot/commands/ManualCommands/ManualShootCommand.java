package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ManualShootCommand extends Command {
  private Shooter shooter;
  private Transport transport;

  public ManualShootCommand(Shooter shooter, Transport transport) {
    addRequirements(shooter, transport);
    this.shooter = shooter;
    this.transport = transport;
  }

  @Override
  public void initialize() {
    shooter.setWarmUpSpeed();
    transport.setFast();
  }
  
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }
}
