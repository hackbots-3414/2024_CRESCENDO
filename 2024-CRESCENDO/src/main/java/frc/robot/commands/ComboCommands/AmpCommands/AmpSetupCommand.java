package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AmpSetupCommand extends Command {
  private Elevator elevator;
  private Shooter shooter;
  private ShooterPivot pivot;
  private boolean continueAnyways;

  public AmpSetupCommand(Elevator elevator, Shooter shooter, ShooterPivot pivot) {
    this(elevator, shooter, pivot, false);
  }

  public AmpSetupCommand(Elevator elevator, Shooter shooter, ShooterPivot pivot, boolean continueAnyways) {
    addRequirements(elevator, shooter, pivot);
    this.elevator = elevator;
    this.shooter = shooter;
    this.pivot = pivot;
    this.continueAnyways = continueAnyways;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(PositionConstants.AmpPresets.elevator);
    shooter.stopMotor();
    pivot.stow();
  }
  
  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint() && !continueAnyways;
  }
}