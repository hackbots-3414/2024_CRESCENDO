package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ElevatorCommand extends Command {
    public enum ElevatorPresets {STOW, AMP, TRAP, TEST;}

    Elevator elevator;
    ShooterPivot shooterPivot;
    ElevatorPresets selector;

    public ElevatorCommand(Elevator elevator, ShooterPivot shooterPivot, ElevatorPresets selector) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.selector = selector;
    }

    @Override
    public void execute() {
        switch (selector) {
            case STOW:
                elevator.setElevatorPosition(PositionConstants.stowPresets.elevator);
                // shooterPivot.setPivotPosition(PositionConstants.stowPresets.shooter);
                break;
            case AMP:
                elevator.setElevatorPosition(PositionConstants.ampPresets.elevator);
                // shooterPivot.setPivotPosition(PositionConstants.ampPresets.shooter);
                break;
            case TRAP:
                elevator.setElevatorPosition(PositionConstants.trapPresets.elevator);
                // shooterPivot.setPivotPosition(PositionConstants.trapPresets.shooter);
                break;
            case TEST:
                elevator.setElevatorPosition(PositionConstants.testPresets.elevator);
        }
    }
}