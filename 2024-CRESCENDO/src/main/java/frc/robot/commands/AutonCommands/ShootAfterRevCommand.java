package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShootAfterRevCommand extends Command {
    Shooter shooter;
    Transport transport;
    double velocity;

    public ShootAfterRevCommand(Shooter shooter, Transport transport, double velocity) {
        addRequirements(shooter, transport);
        this.shooter = shooter;
        this.transport = transport;
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity);
        if (shooter.shooterAtSpeed()) {
            transport.setMotor(TransportConstants.transportSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
    }
}
