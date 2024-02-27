package frc.robot.commands.AutonCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AutoScoreCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    Shooter shooter;
    Transport transport;

    Supplier<AimOutputContainer> aimSupplier;
    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    Command revShooterCommand;
    Command elevatorCommand;
    Command shootAfterRevCommand;

    
    double shootWaitSeconds = 0.5;
    double shootWaitTicks = shootWaitSeconds/0.02;
    double ticks = shootWaitTicks + 5;

    public AutoScoreCommand(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter, Transport transport, Supplier<AimOutputContainer> aimSupplier) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.aimSupplier = aimSupplier;
        this.transport = transport;
    }

    @Override
    public void execute() {
        AimOutputContainer output = aimSupplier.get();

        elevatorCommand = new AutoElevatorCommand(elevator, shooterPivot, output.getElevatorHeight(), output.getPivotAngle());
        revShooterCommand = new RevShooterCommand(shooter, transport, output.getShooterVelocity());
        shootAfterRevCommand = new ShootAfterRevCommand(shooter, transport, output.getShooterVelocity());
            
        if (!elevatorCommand.isFinished()) {elevatorCommand.execute();}
        if (!revShooterCommand.isFinished()) {revShooterCommand.execute();}

        if (elevatorCommand.isFinished() && revShooterCommand.isFinished() && output.getIsInRange()) {
            shootAfterRevCommand.execute();
            ticks = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorCommand.end(interrupted);
        shootAfterRevCommand.end(interrupted);
    }
}