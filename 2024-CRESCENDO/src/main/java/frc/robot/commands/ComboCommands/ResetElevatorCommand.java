package frc.robot.commands.ComboCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ResetElevatorCommand extends Command {
    Elevator elevator;
    ShooterPivot pivot;

    public ResetElevatorCommand(Elevator elevator, ShooterPivot pivot) {
        addRequirements(elevator, pivot);
        this.elevator = elevator;
        this.pivot = pivot;
    }

    @Override
    public void execute() {
        elevator.setResetElevatorSpeed();
        pivot.stow();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.getReverseLimit() && pivot.isAtSetpoint();
    }
}