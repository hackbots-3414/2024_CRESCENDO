package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class SpitOutSimpleCommand extends Command {
    Shooter shooter;
    Transport transport;
    double ticks = 0;
    boolean alreadyRanFeed = false;

    public SpitOutSimpleCommand(Shooter shooter, Transport transport) {
        this.shooter = shooter;
        this.transport = transport;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(ShooterConstants.spitOutSpeed);
        ticks = 0;
        alreadyRanFeed = false;
    }

    @Override
    public void execute() {
        if (shooter.shooterAtSpeed() && !alreadyRanFeed) {
            transport.setFast();
            alreadyRanFeed = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (transport.getNoteOnBoard()) {
            ticks = 0;
        } else {
            ticks++;
        }
      
        return ticks > 10;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
    }
}