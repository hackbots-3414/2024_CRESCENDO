package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {
  Intake intake;
  Transport transport;
  Elevator elevator;
  ShooterPivot pivot;
  double intakeSpeed;
  double transportSpeed;

  public IntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot, double intakeSpeed, double transportSpeed) {
    addRequirements(intake, transport);
    this.transport = transport;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.transportSpeed = transportSpeed;
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
    pivot.setPivotPosition(PositionConstants.StowPresets.shooter);
  }

  @Override
  public void execute() {
      if (elevator.isAtSetpoint() && pivot.isAtSetpoint()) {
        intake.setMotor(intakeSpeed);
        transport.setMotor(transportSpeed);
      } else {
        intake.stopMotor();
        transport.stopMotor();
      }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (transport.getIR()) {
      SubsystemManager.getInstance().noteOnBoard = true;
    }
    return transport.getIR();
  }
}