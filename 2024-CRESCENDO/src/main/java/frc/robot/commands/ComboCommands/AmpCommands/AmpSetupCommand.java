package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;

public class AmpSetupCommand extends Command {
  private Elevator elevator;

  public AmpSetupCommand(Elevator elevator) {
    addRequirements(elevator);
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.AmpPresets.elevator);
  }
  
  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }
}