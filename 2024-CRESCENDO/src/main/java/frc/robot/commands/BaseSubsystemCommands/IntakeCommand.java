package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Consumer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {
  Intake intake;
  Transport transport;
  Elevator elevator;
  ShooterPivot pivot;
  Shooter shooter;
  double intakeSpeed;
  double transportSpeed;
  boolean alreadyStarted = false;
  boolean seenNote;
  Logger log = LoggerFactory.getLogger(IntakeCommand.class);

  public IntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot, double intakeSpeed, double transportSpeed, Shooter shooter) {
    addRequirements(intake, transport);
    this.transport = transport;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.transportSpeed = transportSpeed;
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
    pivot.setPivotPosition(PositionConstants.StowPresets.shooter);
    alreadyStarted = false;
    seenNote = false;
    SmartDashboard.putString("Intake Status", "Initialized");
  }

  @Override
  public void execute() {
    if (elevator.isAtSetpoint() && pivot.isAtSetpoint() && !alreadyStarted) {
      intake.setMotor(intakeSpeed);
      transport.setMotor(transportSpeed);
      alreadyStarted = true;
      SmartDashboard.putString("Intake Status", "Started");
    }
    if (transport.getFlyWheelIR() && !seenNote) {
      log.debug("we are backing up the intake now");
      seenNote = true;
      transport.setMotor(-0.2);
      shooter.setMotor(-0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
    shooter.stopMotor();
    if (!interrupted) {
      transport.setNoteOnBoard(true);
    }
  }

  @Override
  public boolean isFinished() {
    if (seenNote && !transport.getFlyWheelIR()) {
      return true;
    }
    return false;
  }
}