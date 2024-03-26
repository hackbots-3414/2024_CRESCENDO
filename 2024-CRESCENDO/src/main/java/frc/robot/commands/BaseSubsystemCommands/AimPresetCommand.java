package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AimPresetCommand extends Command {
    ShooterPivot shooterPivot;
    Transport transport;
    Supplier<Alliance> aSupplier;
    Supplier<AimOutputContainer> aimSupplier;

    boolean blueSide = false;
    AimOutputContainer output;

    public AimPresetCommand(ShooterPivot shooterPivot, Transport transport, Supplier<Alliance> aSupplier, Supplier<AimOutputContainer> aimSupplier) {
        this.shooterPivot = shooterPivot;
        this.transport = transport;
        this.aSupplier = aSupplier;
        this.aimSupplier = aimSupplier;
    }

    @Override 
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue;
    }

    @Override
    public void execute() {
        if (transport.getNoteOnBoard()) {
            output = aimSupplier.get();
            shooterPivot.setPivotPosition(output.getPivotAngle());
        }
    }
}