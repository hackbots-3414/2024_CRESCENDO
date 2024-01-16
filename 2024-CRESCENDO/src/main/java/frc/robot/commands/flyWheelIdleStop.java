package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hood;

public class flyWheelIdleStop extends Command {

  Hood m_Hood;

  public flyWheelIdleStop(Hood m_Hood) {
    this.m_Hood = m_Hood;
  }

  @Override
  public void initialize() {
    if (m_Hood.getMotorSpeed() != 0) {
      m_Hood.stopMotor();
    } else {
      m_Hood.setIdle();
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
