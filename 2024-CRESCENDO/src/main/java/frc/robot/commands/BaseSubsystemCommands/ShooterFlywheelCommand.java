package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterFlywheelCommand extends Command {
    Transport transport;
    Shooter shooter;
    boolean running = false;

    public ShooterFlywheelCommand(Shooter shooter, Transport transport) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.transport = transport;
    }

    @Override
    public void initialize() {
        running = false;
    }

    @Override
    public void execute() {
        if (transport.getNoteOnBoard()) {
            if (!running) {
                running = true;
                shooter.setWarmUpSpeed();
            }
        } else {
            if (running) {
                shooter.stopMotor();
            }
        }
    }
}