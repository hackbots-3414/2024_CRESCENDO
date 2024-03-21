package frc.robot.commands.ComboCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class SpitOutCommand extends Command {
    Shooter shooter;
    Transport transport;

    public SpitOutCommand(Shooter shooter, Transport transport) {
        this.shooter = shooter;
        this.transport = transport;
    }

    @Override
    public void execute() {
        shooter.setVelocity(ShooterConstants.warmUpSpeed);
        transport.setSlow();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
    }
}
