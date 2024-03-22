package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;

public class ManualWinchCommand extends Command {
    Winch winch;
    boolean goingUp;

    public ManualWinchCommand(Winch winch, boolean goingUp) {
        addRequirements(winch);
        this.winch = winch;
        this.goingUp = goingUp;
    }

    @Override
    public void execute() {
        if (goingUp) {
            winch.setClimbUpSpeed();
        } else {
            winch.setClimbDownSpeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        winch.stopMotor();
    }
}