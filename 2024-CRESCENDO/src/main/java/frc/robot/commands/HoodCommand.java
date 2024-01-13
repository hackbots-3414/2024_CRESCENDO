package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class HoodCommand extends Command {
  private Hood m_Hood; 
  public HoodCommand(Hood m_Hood) {
    addRequirements(m_Hood);
    this.m_Hood = m_Hood;
  }

  @Override
  public void initialize() {
    m_Hood.startMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Hood.setIdle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
