package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // private static final double kSimLoopPeriod = 0.005; // 5 ms
    // private Notifier m_simNotifier = null;
    // private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private Field2d field;
    
		public enum AutonChoice {
        Test1("TestHallway");
        // Test2("TestAuto1"),
        // Test3("TestAuto2");

        public final String value;

        AutonChoice(String value) {
            this.value = value;
        }
    }

    private HashMap<String, Command> eventMarkers = new HashMap<>();

    private PhotonVision photonVision;

    private void initPhotonVision() {
        photonVision = new PhotonVision();
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        initPhotonVision();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        initPhotonVision();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0; // *******************************************************************
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        eventMarkers.put("Shoot", new InstantCommand());
        eventMarkers.put("Intake", new InstantCommand());
        NamedCommands.registerCommands(eventMarkers);

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(2, 0, 0),
                                            new PIDConstants(2, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig(true, true)),
            ()->true, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(AutonChoice pathName) {
        return new PathPlannerAuto(pathName.value);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    // private void startSimThread() {
    //     m_lastSimTime = Utils.getCurrentTimeSeconds();

    //     /* Run simulation at a faster rate so PID gains behave more reasonably */
    //     m_simNotifier = new Notifier(() -> {
    //         final double currentTime = Utils.getCurrentTimeSeconds();
    //         double deltaTime = currentTime - m_lastSimTime;
    //         m_lastSimTime = currentTime;

    //         /* use the measured time delta, get battery voltage from WPILib */
    //         updateSimState(deltaTime, RobotController.getBatteryVoltage());
    //     });
    //     m_simNotifier.startPeriodic(kSimLoopPeriod);
    // }

    private void updateOdometry() {
        Optional<EstimatedRobotPose> leftPoseMaybe = photonVision.getGlobalPoseFromLeft();
        Optional<EstimatedRobotPose> rightPoseMaybe = photonVision.getGlobalPoseFromRight();

        SmartDashboard.putBoolean("SeesRight", rightPoseMaybe.isPresent());
        SmartDashboard.putBoolean("SeesLeft", leftPoseMaybe.isPresent());

        if (leftPoseMaybe.isPresent()) {
            EstimatedRobotPose leftPose = leftPoseMaybe.get();
            SmartDashboard.putString("Left", leftPose.estimatedPose.toString());
            addVisionMeasurement(leftPose.estimatedPose.toPose2d(), leftPose.timestampSeconds);
        }
        if (rightPoseMaybe.isPresent()) {
            EstimatedRobotPose rightPose = rightPoseMaybe.get();
            SmartDashboard.putString("Right", rightPose.estimatedPose.toString());
            addVisionMeasurement(rightPose.estimatedPose.toPose2d(), rightPose.timestampSeconds);
        }
        Pose2d estimatedPose = m_odometry.update(getRotation3d().toRotation2d(), m_modulePositions);
        SmartDashboard.putString("ROBOTPOSE", estimatedPose.toString());
        SmartDashboard.putNumber("ROBOTX", estimatedPose.getX());
        SmartDashboard.putNumber("ROBOTY", estimatedPose.getY());
    }

    @Override
    public void periodic() {
        updateOdometry();
        
    }
}
