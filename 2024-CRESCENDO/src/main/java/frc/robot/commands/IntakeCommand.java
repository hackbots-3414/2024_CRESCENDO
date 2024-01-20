package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {

  Intake m_Intake;
  Transport m_Transport;
  double speed;

  public IntakeCommand(Transport m_Transport, Intake m_Intake, double speed) {
    addRequirements(m_Transport, m_Intake);
    this.m_Transport = m_Transport;
    this.m_Intake = m_Intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    m_Intake.setMotor(speed);
    m_Transport.startMotorforward();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopMotor();
    m_Transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return m_Transport.getIR();
  }
}
