package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterEject extends Command {
    Shooter shooter;
    Transport transport;
    double ticks = 0;

    public ShooterEject(Shooter shooter, Transport transport) {
        this.shooter = shooter;
        this.transport = transport;
        addRequirements(shooter, transport);
    }

    @Override
    public void initialize() {
        shooter.setShooterEject();
        transport.setFast();
        ticks = 0;
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