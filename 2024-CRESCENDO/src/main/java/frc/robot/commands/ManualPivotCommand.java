package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class ManualPivotCommand extends Command {
    ShooterPivot pivot;
    double speed;

    public ManualPivotCommand(ShooterPivot pivot, double speed) {
        addRequirements(pivot);
        this.pivot = pivot;
        this.speed = speed;
    }

    @Override
    public void execute() {
        pivot.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.set(0);
    }
}