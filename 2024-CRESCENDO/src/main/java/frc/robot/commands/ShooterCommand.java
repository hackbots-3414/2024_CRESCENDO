package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

  Shooter m_Shooter;
  double speed;

  public ShooterCommand(Shooter m_Shooter, double speed) {
    addRequirements(m_Shooter);
    this.m_Shooter = m_Shooter;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    m_Shooter.setFlywheelVelo(Constants.ShooterConstants.shootVelo);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Shooter.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
