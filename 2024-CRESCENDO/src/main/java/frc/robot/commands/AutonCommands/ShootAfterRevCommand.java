package frc.robot.commands.AutonCommands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShootAfterRevCommand extends Command {
    Shooter shooter;
    Transport transport;
    double velocity;
    Consumer<Boolean> setNoteOnBoard;

    public ShootAfterRevCommand(Shooter shooter, Transport transport, double velocity, Consumer<Boolean> setNoteOnBoard) {
        addRequirements(shooter, transport);
        this.shooter = shooter;
        this.transport = transport;
        this.velocity = velocity;
        this.setNoteOnBoard = setNoteOnBoard;
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity);
        if (shooter.shooterAtSpeed()) {
            transport.setMotor(TransportConstants.transportSpeed);
            setNoteOnBoard.accept(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
    }
}
