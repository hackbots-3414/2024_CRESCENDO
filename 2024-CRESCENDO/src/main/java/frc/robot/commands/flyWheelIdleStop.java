package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class flyWheelIdleStop extends Command {

  Hood m_hood;

  public flyWheelIdleStop(Hood m_Hood) {
    this.m_hood = m_hood;
  }

  @Override
  public void initialize() {
    if (m_hood.getMotorSpeed() != 0) {
      m_hood.stopMotor();
    } else {
      m_hood.setIdle();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
