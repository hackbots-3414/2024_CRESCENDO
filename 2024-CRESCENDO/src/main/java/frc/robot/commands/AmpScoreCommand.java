package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants.AmpPresets;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AmpScoreCommand extends Command {
  private Transport transport;
  private Elevator elevator;
  private ShooterPivot shooterPivot;
  private Command stowElevator;
  private Shooter shooter;

  public AmpScoreCommand(Transport transport, Elevator elevator, Shooter shooter, ShooterPivot shooterPivot) {
    addRequirements(transport, elevator, shooterPivot);
    this.transport = transport;
    this.elevator = elevator;
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    stowElevator = new StowElevatorCommand(elevator, shooterPivot);
  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(AmpPresets.elevator);
    shooterPivot.setPivotPosition(AmpPresets.shooter);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setMotor(-0.1);
    transport.eject();
    shooter.stopMotor();
    transport.stopMotor();
    stowElevator.schedule();
  }
}
