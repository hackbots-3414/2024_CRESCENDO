package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class SpitOutCommand extends Command {
    ShooterPivot shooterPivot;
    Shooter shooter;
    Transport transport;

    Supplier<Alliance> aSupplier;

    boolean blueSide = false;
    private boolean lateInit;

    boolean alreadyRanFeed = false;
    boolean alreadyRanShooter = false;

    private int ticksSinceShooterSet;
    private int waitTicks = 35;

    ShooterCommand shooterCommand;

    public SpitOutCommand(ShooterPivot shooterPivot, Shooter shooter, Transport transport, Supplier<Alliance> aSupplier) {
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.transport = transport;
        this.aSupplier = aSupplier;
        shooterCommand = new ShooterCommand(shooter, transport, shooterPivot);
        addRequirements(shooter, shooterPivot, transport);
    }

    @Override 
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue;
        shooterCommand.initialize();
        alreadyRanFeed = false;
        alreadyRanShooter = false;
        lateInit = false;
        ticksSinceShooterSet = 0;
    }

    @Override
    public void execute() {
        if (!lateInit) {
            shooterPivot.setPivotPosition(0.06);
            shooter.setFeedSpeed();
            lateInit = true;
        } else {
            ticksSinceShooterSet ++;
        }
        if (ticksSinceShooterSet > waitTicks) {
            executeShooter();
        }
    }

    @Override
    public boolean isFinished() {
        return shooterCommand.isFinished();
    }

    public void executeShooter() {
        if(!alreadyRanShooter){
            shooter.setFeedSpeed();
            alreadyRanShooter = true;
        }
      
        if(!alreadyRanFeed){
            transport.setFast();
            alreadyRanFeed = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        transport.stopMotor();
    }
}
