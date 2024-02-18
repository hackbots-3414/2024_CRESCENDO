package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;

public class WinchCommand extends Command {
    Winch winch;
    double position;

    public WinchCommand(Winch winch, double position) {
        addRequirements(winch);
        this.winch = winch;
        this.position = position;
    }

    @Override
    public void execute() {
        winch.setMotorPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        winch.stopMotor();
    }
}