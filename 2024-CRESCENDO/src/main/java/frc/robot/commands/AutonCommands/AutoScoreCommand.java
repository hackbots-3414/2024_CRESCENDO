package frc.robot.commands.AutonCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimHelper;
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

    Supplier<Alliance> aSupplier;
    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    Command shootCommand;
    Command elevatorCommand;
    Command shootAfterRevCommand;
    Command intakeCommand;

    public AutoScoreCommand(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter, Transport transport, Command intakeCommand, Supplier<Alliance> aSupplier, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.aSupplier = aSupplier;
        this.transport = transport;
        this.intakeCommand = intakeCommand;
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public void execute() {
        // AimOutputContainer output = AimHelper.calculateAimWithMath(poseSupplier.get(), chassisSpeedsSupplier.get(), aSupplier.get() == Alliance.Blue); // WITH MATH
        AimOutputContainer output = AimHelper.calculateAimLookupTable(poseSupplier.get(), aSupplier.get() == Alliance.Blue); // LOOKUP TABLE

        elevatorCommand = new AutoElevatorCommand(elevator, shooterPivot, output.getElevatorHeight(), output.getPivotAngle());
        shootCommand = new RevShooterCommand(shooter, transport, output.getShooterVelocity());
        shootAfterRevCommand = new ShootAfterRevCommand(shooter, transport, output.getShooterVelocity());

        if (!elevatorCommand.isFinished()) {elevatorCommand.execute();}
        if (!shootCommand.isFinished()) {shootCommand.execute();}
        if (elevatorCommand.isFinished() && shootCommand.isFinished()) {shootAfterRevCommand.execute();}

        if (output.getIsInRange()) {
            intakeCommand.end(true);
            
            if (!elevatorCommand.isFinished()) {elevatorCommand.execute();}
            if (!shootCommand.isFinished()) {shootCommand.execute();}
            if (elevatorCommand.isFinished() && shootCommand.isFinished()) {shootAfterRevCommand.execute();}
        } else {
            elevatorCommand.end(true);
            shootCommand.end(true);
            shootAfterRevCommand.end(true);            

            if (!intakeCommand.isFinished()) {intakeCommand.initialize();}
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeCommand.end(interrupted);
        elevatorCommand.end(interrupted);
        shootAfterRevCommand.end(interrupted);
    }
}