package frc.robot.commands.BaseSubsystemCommands;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
  // private Logger log = LoggerFactory.getLogger(AutoIntakeCommand.class);
  private boolean seenNote;

  private boolean alreadyStarted;


  public AutoIntakeCommand(Transport transport, Intake intake, Elevator elevator, ShooterPivot pivot, Shooter shooter) {
    addRequirements(intake, transport, elevator, shooter, pivot);
    this.transport = transport;
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    elevator.stow();
    pivot.stow();
    shooter.stopMotor();
    seenNote = false;
    alreadyStarted = false;
  }


  @Override
  public void execute() {
    if (elevator.isAtSetpoint() && pivot.isAtSetpoint()) {
      if (!alreadyStarted) {
        intake.setFast();
        transport.setFast();
        alreadyStarted = true;
      }
    }
    if (transport.getFlyWheelIR() && !seenNote) {
      transport.setBackup();
      shooter.setBackup();
      intake.setEject();
      seenNote = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
    if (DriverStation.isAutonomous()) {
      shooter.setWarmUpSpeed();
    } else {
      shooter.stopMotor();
    }
  }

  @Override
  public boolean isFinished() {
    return transport.getTransportIR() && seenNote && !transport.getFlyWheelIR();
  }
}