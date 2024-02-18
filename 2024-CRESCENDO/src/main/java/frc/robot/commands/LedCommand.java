package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand extends Command {

  private LedSubsystem m_LedSubsystem;
  double m_ledColor;

  public LedCommand(LedSubsystem m_LedSubsystem, double ledColor) {
    this.m_LedSubsystem = m_LedSubsystem;
    this.m_ledColor = ledColor;
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void execute() {
    m_LedSubsystem.setColor(m_ledColor);
  }
}
