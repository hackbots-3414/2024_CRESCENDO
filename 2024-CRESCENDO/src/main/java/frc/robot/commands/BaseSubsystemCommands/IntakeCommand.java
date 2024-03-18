package frc.robot.commands.BaseSubsystemCommands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  boolean alreadyStarted = false;
  boolean alreadyStartedSlowed;
  Logger log = LoggerFactory.getLogger(IntakeCommand.class);


  public IntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot) {
    addRequirements(intake, transport);
    this.transport = transport;
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
  // SmartDashboard.putString("INTAKE FAST", "Initialized");

  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
    pivot.setPivotPosition(PositionConstants.StowPresets.shooter);
    alreadyStarted = false;
    alreadyStartedSlowed = false;

  }


  @Override
  public void execute() {
    if (elevator.isAtSetpoint() && pivot.isAtSetpoint()) {
      if (!alreadyStarted) {
        intake.setFast();
        transport.setFast();
        alreadyStarted = true;
        // SmartDashboard.putString("INTAKE FAST", "FAST");

      }  
      if (!alreadyStartedSlowed && transport.getTransportIR()) {
        intake.setSlow();
        transport.setSlow();
        alreadyStartedSlowed= true;
        // SmartDashboard.putString("INTAKE FAST", "SLOW");

      }  

    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
    
  }

  @Override
  public boolean isFinished() {
    return transport.getFlyWheelIR() && transport.getTransportIR();
  }
}