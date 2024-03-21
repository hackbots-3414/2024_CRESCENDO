package frc.robot.commands.ComboCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class SpitOutCommand extends Command {
    Shooter shooter;
    Transport transport;
    Intake intake;

    public SpitOutCommand(Shooter shooter, Transport transport, Intake intake) {
        this.shooter = shooter;
        this.transport = transport;
        this.intake = intake;
    }

    @Override
    public void execute() {
        shooter.setVelocity(ShooterConstants.warmUpSpeed);
        transport.setSlow();
        intake.setSlow();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
        intake.stopMotor();
    }
}
