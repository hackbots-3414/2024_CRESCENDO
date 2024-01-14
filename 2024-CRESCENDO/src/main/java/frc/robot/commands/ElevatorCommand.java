package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    public enum ElevatorPresets {
        AMP,
        TRAP,
        CLIMB,
        STOW;
    }

    Elevator elevator = new Elevator();
    ElevatorPresets selector;

    public ElevatorCommand(Elevator elevator, ElevatorPresets selector) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.selector = selector;
    }

    @Override
    public void initialize() {
        elevator.disable();
    }

    @Override
    public void execute() {
        switch (selector) {
            case AMP:
                elevator.setGoal(Constants.ElevatorConstants.ampPreset);
                break;
            case TRAP:
                elevator.setGoal(Constants.ElevatorConstants.trapPreset);
                break;
            case CLIMB:
                elevator.setGoal(Constants.ElevatorConstants.climbPreset);
                break;
            case STOW:
                elevator.setGoal(Constants.ElevatorConstants.stowPreset);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
