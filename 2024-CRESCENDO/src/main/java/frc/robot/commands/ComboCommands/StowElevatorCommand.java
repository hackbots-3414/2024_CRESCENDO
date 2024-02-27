package frc.robot.commands.ComboCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants.StowPresets;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class StowElevatorCommand extends Command {
  private Elevator elevator;
  private ShooterPivot shooterPivot;

  public StowElevatorCommand(Elevator elevator, ShooterPivot shooterPivot) {
    addRequirements(elevator, shooterPivot);
    this.elevator = elevator;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(StowPresets.elevator);
    shooterPivot.setPivotPosition(StowPresets.shooter);
  }

  @Override
  public boolean isFinished() {
    return elevator.getReverseLimit();
  }
}
