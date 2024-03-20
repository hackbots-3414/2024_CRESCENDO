package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DefaultShooterCommand extends Command {
  Shooter shooter;

  public DefaultShooterCommand(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  @Override
  public void execute() {    
    shooter.setWarmUpSpeed();
  }
}
  