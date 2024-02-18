package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command {
    Elevator elevator;
    double speed;

    public ManualElevatorCommand(Elevator elevator, double speed) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.speed = speed;
    }

    @Override
    public void execute() {
        elevator.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}