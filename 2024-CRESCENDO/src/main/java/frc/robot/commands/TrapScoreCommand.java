package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants.StowPresets;
import frc.robot.Constants.PositionConstants.TrapPresets;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class TrapScoreCommand extends Command {
  private Transport transport;
  private Elevator elevator;
  private ShooterPivot shooterPivot;

  public TrapScoreCommand(Transport transport, Elevator elevator, ShooterPivot shooterPivot) {
    addRequirements(transport, elevator, shooterPivot);
    this.transport = transport;
    this.elevator = elevator;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(TrapPresets.elevator);
    shooterPivot.setPivotPosition(TrapPresets.shooter);
  }

  @Override
  public void end(boolean interrupted) {
    transport.eject();
    elevator.setElevatorPosition(StowPresets.elevator);
    shooterPivot.setPivotPosition(StowPresets.shooter);
  }
}
