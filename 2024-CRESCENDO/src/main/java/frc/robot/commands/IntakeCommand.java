package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

  Intake m_Intake;
  double speed;

  public IntakeCommand(Intake m_Intake, double speed) {
    addRequirements(m_Intake);
    this.m_Intake = m_Intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    m_Intake.setMotor(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
