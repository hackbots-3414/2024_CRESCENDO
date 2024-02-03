package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

  Shooter shooter;
  double speed;

  public ShooterCommand(Shooter shooter, double speed) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    shooter.setMotor(speed);
    shooter.setRunning(true);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    shooter.setRunning(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
