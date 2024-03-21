package frc.robot.commands.AutonCommands;

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

public class AutoIntakeCommand extends Command {
  private Intake intake;
  private Transport transport;
  private Elevator elevator;
  private ShooterPivot pivot;
  private Shooter shooter;
  private Logger log = LoggerFactory.getLogger(AutoIntakeCommand.class);
  private boolean seenNote;
  private boolean ready;


  public AutoIntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot, Shooter shooter) {
    addRequirements(intake, transport);
    this.transport = transport;
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
  // SmartDashboard.putString("INTAKE FAST", "Initialized");

  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
    pivot.setPivotPosition(PositionConstants.StowPresets.shooter);
    shooter.stopMotor();
    seenNote = false;
    ready = false;

  }


  @Override
  public void execute() {
    if (pivot.isAtSetpoint() && elevator.isAtSetpoint()) ready = true;
    if (transport.getFlyWheelIR() && !seenNote && ready) {
      transport.setBackup();
      shooter.setBackup();
      seenNote = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
    shooter.setWarmUpSpeed(); // to prepare us for the next shoot
  }

  @Override
  public boolean isFinished() {
    return transport.getFlyWheelIR() && transport.getTransportIR();
  }
}