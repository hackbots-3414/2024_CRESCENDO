package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AutoPivotCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;
    Shooter shooter;

    double elevatorHeight;
    double shooterAngle;
    Rotation2d drivetrainRotation;
    boolean blueSide = false;
    Supplier<Alliance> aSupplier;

    double velocityPerpendicularGain = 0;

    PIDController pidTurn = new PIDController(0.05, 0, 0);
    Pose2d robotPosition2d;


    public AutoPivotCommand(Elevator elevator, ShooterPivot shooterPivot, CommandSwerveDrivetrain drivetrain, Shooter shooter, Supplier<Alliance> aSupplier) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.aSupplier = aSupplier;
    }

    @Override
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue ? true : false;
    }

    private void recalculate() {
        ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        double velocityPerpendicular = speeds.vyMetersPerSecond;
        double pitchAdd = velocityPerpendicular * velocityPerpendicularGain;

        robotPosition2d = drivetrain.getPose();
        Pose2d speakerPosition =  blueSide ? AprilTags.BlueSpeakerCenter.value.getPose2d() : AprilTags.RedSpeakerCenter.value.getPose2d();
        Pose2d speakerRelative = speakerPosition.relativeTo(robotPosition2d);

        double v = AimConstants.shootSpeed;
        double g = 9.81;
        double x = speakerRelative.getTranslation().getNorm();
        double y = AimConstants.speakerHeight - AimConstants.minimumHeight;

        if (x > AimConstants.minimumDistanceToNotBreakRobot) {
            elevatorHeight = 0.0;
        } else {
            elevatorHeight = AimConstants.clearanceHeight;
            y -= elevatorHeight * Math.sin(AimConstants.elevatorTilt);
            x += elevatorHeight * Math.cos(AimConstants.elevatorTilt);
        }

        double theta = Math.atan((Math.pow(v, 2) - Math.sqrt(Math.pow(v, 4) - g * (g * Math.pow(x, 2) + 2 * y * Math.pow(v, 2)))) / (g * x));
        drivetrain.setInRange(x < Constants.AimConstants.range);
        shooterAngle = runTests(v, theta, g, x, y) && drivetrain.isInRange() ? (theta + pitchAdd) : shooterAngle;

        if (x < AimConstants.shooterInRange) {
            shooter.setMotor(ShooterConstants.shootVelo);
        } else {
            shooter.stopMotor(); 
        }
    }

    @Override
    public void execute() {
        recalculate();
        elevator.setElevatorPosition(elevatorHeight);
        shooterPivot.setPivotPositionFromRad(shooterAngle);

        SmartDashboard.putNumber("SHOOTER ANGLE", shooterAngle);
    }

    private boolean runTests(double velocity, double launchAngle, double gravity, double checkX, double checkY) {
        double initialVelocityX = velocity * Math.cos(launchAngle);
        double initialVelocityY = velocity * Math.sin(launchAngle);
        double timeOfFlight = checkX / initialVelocityX;
        double verticalDistanceTraveled =initialVelocityY * timeOfFlight - 0.5 * gravity * timeOfFlight * timeOfFlight;
        
        return verticalDistanceTraveled - checkY < AimConstants.rangeTolerance;
    }
}