package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ElevatorCommand extends Command {

    public enum ElevatorPresets {STOW,AMP,TRAP;}

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
                elevator.setGoal(PositionConstants.stowPresets.elevator);
                shooterPivot.setGoal(PositionConstants.stowPresets.shooter);
                break;
            case AMP:
                elevator.setGoal(PositionConstants.ampPresets.elevator);
                shooterPivot.setGoal(PositionConstants.ampPresets.shooter);
                break;
            case TRAP:
                elevator.setGoal(PositionConstants.trapPresets.elevator);
                shooterPivot.setGoal(PositionConstants.trapPresets.shooter);
                break;
        }
        elevator.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}