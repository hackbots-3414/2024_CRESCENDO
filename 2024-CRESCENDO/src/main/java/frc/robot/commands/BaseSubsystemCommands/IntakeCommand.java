package frc.robot.commands.BaseSubsystemCommands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class IntakeCommand extends Command {
  Intake intake;
  Transport transport;
  Elevator elevator;
  ShooterPivot pivot;
  boolean alreadyStarted = false;
  boolean alreadyStartedMedium = false;
  boolean alreadyStartedSlowed = false;

  Logger logger = LoggerFactory.getLogger(IntakeCommand.class);


  public IntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot) {
    addRequirements(intake, transport, pivot);
    this.transport = transport;
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void initialize() {
    elevator.stow();
    pivot.stow(); // NOTE this may cause problems with addRequirements, AKSHAY
    alreadyStarted = false;
    alreadyStartedSlowed = false;
    alreadyStartedMedium = false;
    logger.debug("INTAKE INITIALIZE");
  }

  @Override
  public void execute() {
    if (elevator.isAtSetpoint() && pivot.isAtSetpoint()) {
      if (!alreadyStarted) {
        intake.setFast();
        transport.setFast();
        alreadyStarted = true;
        logger.debug("INTAKE FAST");
      }
      if (!alreadyStartedMedium && intake.getIntakeIR()) {
        intake.setMedium();
        transport.setMedium();
        alreadyStartedMedium = true;
        logger.debug("INTAKE MEDIUM");
      }
      if (!alreadyStartedSlowed && transport.getTransportIR()) {
        intake.setSlow();
        transport.setSlow();
        alreadyStartedSlowed = true;
        logger.debug("INTAKE SLOW");
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    logger.debug("INTAKE END");
    intake.stopMotor();
    transport.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return transport.getNoteInPosition();
  }
}