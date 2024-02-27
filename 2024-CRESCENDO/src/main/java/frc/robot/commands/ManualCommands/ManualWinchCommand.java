package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;

public class ManualWinchCommand extends Command {
    Winch winch;
    double speed;

    public ManualWinchCommand(Winch winch, double speed) {
        addRequirements(winch);
        this.winch = winch;
        this.speed = speed;
    }

    @Override
    public void execute() {
        winch.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        winch.stopMotor();
    }
}