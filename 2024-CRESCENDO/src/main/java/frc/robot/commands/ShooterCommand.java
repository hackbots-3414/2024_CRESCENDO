package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

  final Shooter m_Shooter;

  public ShooterCommand(Shooter m_Shooter) {
    this.m_Shooter = m_Shooter;
  }

  @Override
  public void initialize() {
    m_Shooter.startMotor();
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
