package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

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
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class AimRobotCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;

    double elevatorHeight;
    double shooterAngle;
    Rotation2d drivetrainRotation;
  
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;
    Supplier<Alliance> aSupplier;
    boolean blueSide = false;

    Command currentDriveCommand;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    FieldCentricFacingAngle autoRequest = new SwerveRequest.FieldCentricFacingAngle().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withSteerRequestType(SteerRequestType.MotionMagic);

    double velocityParallelGain = 0.0;
    double velocityPerpendicularGain = 0;

    PIDController pidTurn = new PIDController(0.05, 0, 0);
    Pose2d robotPosition2d;


    public AimRobotCommand(Elevator elevator, ShooterPivot shooterPivot, CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Supplier<Alliance> aSupplier) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.aSupplier = aSupplier;
    }

    private void recalculate() {
        ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        double velocityParallel = speeds.vxMetersPerSecond;
        double velocityPerpendicular = speeds.vyMetersPerSecond;
        double yawAdd = velocityParallel * velocityParallelGain;
        double pitchAdd = velocityPerpendicular * velocityPerpendicularGain;

        robotPosition2d = drivetrain.getPose();
        Pose2d speakerPosition =  blueSide ? AprilTags.BlueSpeakerCenter.value.getPose2d() : AprilTags.RedSpeakerCenter.value.getPose2d();
        Pose2d speakerRelative = speakerPosition.relativeTo(robotPosition2d);

        drivetrainRotation = speakerPosition.getTranslation().minus(robotPosition2d.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(yawAdd));

        double v = AimConstants.shootSpeed;
        double g = 9.81;
        double x = speakerRelative.getTranslation().getNorm();
        double y = AimConstants.speakerHeight - AimConstants.minimumHeight;
        
        SmartDashboard.putNumber("GOAL HEIGHT", y);
        SmartDashboard.putNumber("DISTANCE FROM TARGET", x);

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
    }

    @Override
    public void execute() {
        blueSide = aSupplier.get() == Alliance.Blue;
        recalculate();
        elevator.setElevatorPosition(elevatorHeight);
        shooterPivot.setPivotPositionFromRad(shooterAngle);
        double compensate = blueSide ? -360 : 0;
        double measurement = robotPosition2d.getRotation().getDegrees() > 0 ? robotPosition2d.getRotation().getDegrees() + compensate : robotPosition2d.getRotation().getDegrees();
        double setpoint = drivetrainRotation.getDegrees() > 0 ? drivetrainRotation.getDegrees() + compensate : drivetrainRotation.getDegrees();

        currentDriveCommand = drivetrain.applyRequest(() -> driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withVelocityY(-ySupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withRotationalRate((rSupplier.get() > 0.3 || rSupplier.get() < -0.3) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
                                : (pidTurn.calculate(measurement, setpoint) * Constants.SwerveConstants.maxAngleVelocity)));

        currentDriveCommand.schedule();

        SmartDashboard.putNumber("SHOOTER ANGLE", shooterAngle);
        SmartDashboard.putNumber("SHOOTER FORCE", AimConstants.shootSpeed);
    }

    @Override
    public void end(boolean interrupted) {currentDriveCommand.cancel();}

    private boolean runTests(double velocity, double launchAngle, double gravity, double checkX, double checkY) {
        double initialVelocityX = velocity * Math.cos(launchAngle);
        double initialVelocityY = velocity * Math.sin(launchAngle);
        double timeOfFlight = checkX / initialVelocityX;
        double verticalDistanceTraveled =initialVelocityY * timeOfFlight - 0.5 * gravity * timeOfFlight * timeOfFlight;
        
        return verticalDistanceTraveled - checkY < AimConstants.rangeTolerance;
    }
}