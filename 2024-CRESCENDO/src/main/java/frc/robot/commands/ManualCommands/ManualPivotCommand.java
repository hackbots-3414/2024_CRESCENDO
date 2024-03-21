package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class ManualPivotCommand extends Command {
    ShooterPivot pivot;
    boolean goingUp;

    public ManualPivotCommand(ShooterPivot pivot, boolean goingUp) {
        addRequirements(pivot);
        this.pivot = pivot;
        this.goingUp = goingUp;
    }

    @Override
    public void execute() {
        if (goingUp) {
            pivot.setPivotUpSpeed();
        } else {
            pivot.setPivotDownSpeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}