package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AimPresetCommand extends Command {
    ShooterPivot shooterPivot;
    Shooter shooter;
    Supplier<Alliance> aSupplier;
    Supplier<AimOutputContainer> aimSupplier;
    Supplier<Boolean> hasNote;

    boolean blueSide = false;
    AimOutputContainer output;

    public AimPresetCommand(ShooterPivot shooterPivot, Shooter shooter, Supplier<Alliance> aSupplier, Supplier<AimOutputContainer> aimSupplier, Supplier<Boolean> hasNote) {
        this.shooterPivot = shooterPivot;
        this.aSupplier = aSupplier;
        this.aimSupplier = aimSupplier;
        this.hasNote = hasNote;
    }

    @Override 
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue;
    }

    @Override
    public void execute() {
        output = aimSupplier.get();
        shooterPivot.setPivotPosition(output.getPivotAngle());
        shooter.setMaxSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        shooterPivot.stow();
    }

    @Override
    public boolean isFinished() {
        return !hasNote.get();
    }
}