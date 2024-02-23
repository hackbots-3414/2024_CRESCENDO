package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class RevShooterCommand extends Command {
    Shooter shooter;
    Transport transport;
    double velocity;

    public RevShooterCommand(Shooter shooter, Transport transport, double velocity) {
        this.shooter = shooter;
        this.transport = transport;
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        if (transport.getIR() == true) {
            transport.stopMotor();
            shooter.setVelocity(velocity);
        } else {
            transport.setMotor(TransportConstants.transportEjectSpeed);
            shooter.setMotor(ShooterConstants.shooterIntakeSpeed);
        }
    }
    @Override
    public boolean isFinished() {
        return shooter.shooterAtSpeed();
    }
}
