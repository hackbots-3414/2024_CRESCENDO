package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command {
    Elevator elevator;
    boolean goingUp;

    public ManualElevatorCommand(Elevator elevator, boolean goingUp) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.goingUp = goingUp;
    }

    @Override
    public void execute() {
        if (goingUp) {
            elevator.setElevatorUpSpeed();
        } else {
            elevator.setElevatorDownSpeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}