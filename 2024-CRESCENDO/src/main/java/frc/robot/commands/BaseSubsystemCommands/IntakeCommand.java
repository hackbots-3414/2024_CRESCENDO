package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {
  Intake intake;
  Transport transport;
  Elevator elevator;
  ShooterPivot pivot;
  double intakeSpeed;
  double transportSpeed;
  Consumer<Boolean> setNoteIsOnBoard;

  public IntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot, double intakeSpeed, double transportSpeed, Consumer<Boolean> setNoteIsOnBoard) {
    addRequirements(intake, transport);
    this.transport = transport;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.transportSpeed = transportSpeed;
    this.elevator = elevator;
    this.pivot = pivot;
    this.setNoteIsOnBoard = setNoteIsOnBoard;
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
      setNoteIsOnBoard.accept(true);
    }
    return transport.getIR();
  }
}