package frc.robot.commands.AutonCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BaseSubsystemCommands.ShooterCommand;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AutoScoreCommand extends Command {
    private Elevator elevator;
    private ShooterPivot shooterPivot;
    private Shooter shooter;
    private Transport transport;

    private Supplier<AimOutputContainer> aimSupplier;
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private Command revShooterCommand;
    private Command elevatorCommand;
    private Command shootAfterRevCommand;

    private double shootWaitSeconds = 0.5;
    private double shootWaitTicks = shootWaitSeconds/0.02;
    private double ticks = shootWaitTicks + 5;

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
        shootAfterRevCommand = new ShooterCommand(shooter, transport).withTimeout(shootWaitSeconds);
            
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