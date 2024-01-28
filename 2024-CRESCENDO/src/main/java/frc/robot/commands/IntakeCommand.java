package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {

  Intake intake;
  Transport transport;
  double intakeSpeed;
  double transportSpeed;

  public IntakeCommand(Transport transport, Intake intake, double intakeSpeed, double transportSpeed) {
    addRequirements(transport, intake);
    this.transport = transport;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.transportSpeed = transportSpeed;
  }

  @Override
  public void initialize() {
    intake.setMotor(intakeSpeed);
    intake.setRunning(true);
    // m_Transport.setMotor(transportSpeed);
    transport.setRunning(true);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    intake.setRunning(false);
    // transport.stopMotor();
    transport.setRunning(false);
  }

  @Override
  public boolean isFinished() {
    // return m_Transport.getIR() && intakeSpeed > 0;
    return intake.getForwardLimit();
  }
}