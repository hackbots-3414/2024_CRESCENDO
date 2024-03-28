package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class AmpSetupCommand extends Command {
  private Elevator elevator;
  private Shooter shooter;

  public AmpSetupCommand(Elevator elevator, Shooter shooter) {
    addRequirements(elevator, shooter);
    this.elevator = elevator;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.AmpPresets.elevator);
    shooter.stopMotor();
  }
  
  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }
}