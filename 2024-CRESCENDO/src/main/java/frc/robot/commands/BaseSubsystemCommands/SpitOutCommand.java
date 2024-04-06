// package frc.robot.commands.BaseSubsystemCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.Transport;

// public class SpitOutCommand extends Command {
//     Shooter shooter;
//     Transport transport;
//     double ticks = 0;
//     boolean alreadyRanFeed = false;

//     public SpitOutCommand(Shooter shooter, ShooterPivot pivot, Transport transport) {
//         this.shooter = shooter;
//         this.transport = transport;
//     }

//     @Override
//     public void initialize() {
//         shooter.setVelocity(ShooterConstants.spitOutSpeed);
//         ticks = 0;
//         alreadyRanFeed = false;
//     }

//     @Override
//     public void execute() {
//         if (shooter.shooterAtSpeed() && !alreadyRanFeed) {
//             transport.setFast();
//             alreadyRanFeed = true;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (transport.getNoteOnBoard()) {
//             ticks = 0;
//         } else {
//             ticks++;
//         }
      
//         return ticks > 10;
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         shooter.stopMotor();
//         transport.stopMotor();
//     }
// }



package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class SpitOutCommand extends Command {
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;
    Shooter shooter;
    Transport transport;

    boolean drivetrainAtGoal;
  
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;
    Supplier<Alliance> aSupplier;

    boolean blueSide = false;

    boolean alreadyRanFeed = false;
    boolean alreadyRanShooter = false;

    ShooterCommand shooterCommand;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    PIDController thetaController = new PIDController(0.8, 0, 0);

    public SpitOutCommand(ShooterPivot shooterPivot, Shooter shooter, Transport transport, CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Supplier<Alliance> aSupplier) {
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.transport = transport;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.aSupplier = aSupplier;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(SwerveConstants.pidTurnTolerance);
        shooterCommand = new ShooterCommand(shooter, transport);
        addRequirements(shooter, shooterPivot, transport, drivetrain);
    }

    @Override 
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue;
        shooterCommand.initialize();
        alreadyRanFeed = false;
        alreadyRanShooter = false;
    }

    @Override
    public void execute() {
        Pose2d robotPosition = drivetrain.getPose();

        Pose2d speakerPose = blueSide ? AimConstants.blueSpeakerPos.transformBy(new Transform2d(0, 1, Rotation2d.fromDegrees(0))) : AimConstants.redSpeakerPos;
        Pose2d drivetrainPose = drivetrain.getPose();

        Rotation2d output = speakerPose.getTranslation().minus(drivetrainPose.getTranslation()).getAngle();


        shooterPivot.setPivotPosition(PivotConstants.forwardSoftLimitThreshold);
        double robotRotation = robotPosition.getRotation().getRadians();
        double targetRotation = output.getRadians();

        drivetrain.setControl(driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
                            .withVelocityY(ySupplier.get() * SwerveConstants.maxDriveVelocity)
                            .withRotationalRate((rSupplier.get() > 0.2 || rSupplier.get() < -0.2) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
                            : (thetaController.calculate(robotRotation, targetRotation) * Constants.SwerveConstants.maxAngleVelocity)));

        SmartDashboard.putBoolean("theta controller at setpoint", thetaController.atSetpoint());

        if (thetaController.atSetpoint()) {
            executeShooter();
        } else {
            shooter.setFeedSpeed();
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
      
        if(shooter.shooterAtSpeed() && !alreadyRanFeed){
            if (shooterPivot != null && !shooterPivot.isAtSetpoint()) { // todo this logic makes no sense, it will fail at some point?
              return;
            }
            transport.setFast();
            alreadyRanFeed = true;
        }
    }
}
