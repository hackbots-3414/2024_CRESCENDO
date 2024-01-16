package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    public enum ElevatorPresets {AMP,TRAP;}

    Elevator elevator;
    ElevatorPresets selector;

    public ElevatorCommand(Elevator elevator, ElevatorPresets selector) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.selector = selector;
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
        }
        elevator.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
