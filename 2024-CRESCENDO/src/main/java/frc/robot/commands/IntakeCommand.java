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
    // addRequirements(intake);
    this.transport = transport;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.transportSpeed = transportSpeed;
  }

  @Override
  public void initialize() {
    intake.setMotor(intakeSpeed);
    transport.setMotor(transportSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return transport.getIR();
  }
}