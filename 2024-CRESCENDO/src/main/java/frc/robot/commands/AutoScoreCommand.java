package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimHelper;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AutoScoreCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;
    Shooter shooter;
    Transport transport;

    boolean blueSide = false;
    Supplier<Alliance> aSupplier;

    Pose2d robotPosition2d;

    Command shootCommand;
    Command intakeCommand;

    public AutoScoreCommand(Elevator elevator, ShooterPivot shooterPivot, CommandSwerveDrivetrain drivetrain, Shooter shooter, Transport transport, Command intakeCommand, Supplier<Alliance> aSupplier) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.aSupplier = aSupplier;
        this.transport = transport;
        this.intakeCommand = intakeCommand;
    }

    @Override
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue ? true : false;
    }

    @Override
    public void execute() {
        // ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        Pose2d robotPosition = drivetrain.getPose();
        // AimOutputContainer output = AimHelper.calculateAimWithMath(robotPosition, speeds, blueSide); // WITH MATH
        AimOutputContainer output = AimHelper.calculateAimLookupTable(robotPosition, blueSide); // LOOKUP TABLE

        elevator.setElevatorPosition(output.getElevatorHeight());
        shooterPivot.setPivotPosition(output.getPivotAngle());

        if (output.getIsInRange()) {
            intakeCommand.cancel();;
            shootCommand = new ShooterCommand(shooter, transport, output.getShooterVelocity());
            shootCommand.schedule();
        } else {
            shootCommand.cancel();
            intakeCommand.schedule();
        }
    }
}