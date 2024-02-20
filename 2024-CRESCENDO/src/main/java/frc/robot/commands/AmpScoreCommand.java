package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants.AmpPresets;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AmpScoreCommand extends Command {
  private Transport transport;
  private Elevator elevator;
  private ShooterPivot shooterPivot;
  private Command stowElevator;

  public AmpScoreCommand(Transport transport, Elevator elevator, ShooterPivot shooterPivot) {
    addRequirements(transport, elevator, shooterPivot);
    this.transport = transport;
    this.elevator = elevator;
    this.shooterPivot = shooterPivot;
    stowElevator = new StowElevatorCommand(elevator, shooterPivot);
  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(AmpPresets.elevator);
    shooterPivot.setPivotPosition(AmpPresets.shooter);
  }

  @Override
  public void end(boolean interrupted) {
    transport.eject();
    stowElevator.schedule();
  }
}
