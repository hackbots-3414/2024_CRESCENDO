package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ResetElevatorCommand extends Command {
    Elevator elevator;
    ShooterPivot pivot;
    SequentialCommandGroup postZeroSequence = new SequentialCommandGroup();

    public ResetElevatorCommand(Elevator elevator, ShooterPivot pivot) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.pivot = pivot;
        postZeroSequence.addCommands(new ElevatorCommand(elevator, pivot, ElevatorPresets.RESET).withTimeout(0.3), new ElevatorCommand(elevator, pivot, ElevatorPresets.STOW).withTimeout(0.3));
    }

    @Override
    public void execute() {
        elevator.set(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        postZeroSequence.schedule();
    }

    @Override
    public boolean isFinished() {
        return elevator.getReverseLimit();
    }
}