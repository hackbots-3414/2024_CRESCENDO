package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class AutoElevatorCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    double elevatorPos;
    double pivotPos;

    public AutoElevatorCommand(Elevator elevator, ShooterPivot shooterPivot, double elevatorPos, double pivotPos) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.elevatorPos = elevatorPos;
        this.pivotPos = pivotPos;
    }

    @Override
    public void execute() {
        elevator.setElevatorPosition(elevatorPos);
        shooterPivot.setPivotPosition(pivotPos);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetpoint() && shooterPivot.isAtSetpoint();
    }
}