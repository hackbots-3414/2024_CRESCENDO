package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AmpSetupCommand extends Command {
  private Elevator elevator;
  private double elevatorGoal = Constants.PositionConstants.AmpPresets.elevator;

  public AmpSetupCommand(Elevator elevator) {
    addRequirements(elevator);
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(elevatorGoal);
  }
  
  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }
}