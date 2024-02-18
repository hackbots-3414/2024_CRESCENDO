package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  Shooter shooter;

  public ShooterCommand(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    shooter.setVelocity(Constants.ShooterConstants.shootVelo);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }
}
