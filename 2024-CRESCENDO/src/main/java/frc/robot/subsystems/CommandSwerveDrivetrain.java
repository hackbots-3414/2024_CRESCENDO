package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.util.VisionHelpers;
import frc.robot.util.VisionHelpers.TimestampedVisionUpdate;
import frc.robot.Constants.DebugConstants;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.002; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field = new Field2d();
    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    private Pose2d estimatedPose;
    private boolean isInRange;
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.estimatedPose = new Pose2d();
        if (Robot.isSimulation()) {
            startSimThread();
        }
        setCurrentLimit(SwerveConstants.driveSupplyCurrentLimit);
        if (DebugConstants.debugMode) SmartDashboard.putData("Field", field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.estimatedPose = new Pose2d();
        if (Robot.isSimulation()) {
            startSimThread();
        }
        setCurrentLimit(SwerveConstants.driveSupplyCurrentLimit);
        if (DebugConstants.debugMode) SmartDashboard.putData("Field", field);
    }

    public Translation2d[] moduleLocations() {
        return m_moduleLocations;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public double getTranslationalRobotSpeed() {
        return Math.sqrt(Math.pow(getCurrentRobotChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getCurrentRobotChassisSpeeds().vyMetersPerSecond, 2));
    }

    public double getRotationalRobotSpeed() {
        return getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
    }

    public Command makeTestAuton() {
        return AutoBuilder.buildAuto("Test Auton");
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getCurrentPose2d() {
        return this.getState().Pose;
    }

    public Pose2d getPose() {
        return estimatedPose;
    }

    public void setCurrentLimit(double limit) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(limit);

        for (int i = 0; i < 3; i++)
            getModule(i).getDriveMotor().getConfigurator().apply(configs);
        getModule(3).getDriveMotor().getConfigurator().apply(configs, 0.015);
    }

     /**
     * Adds vision data to the pose esimation.
     *
     * @param visionData The vision data to add.
     */
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        visionData.forEach(visionUpdate ->
                addVisionMeasurement(visionUpdate.pose(), visionUpdate.timestamp(), visionUpdate.stdDevs()));
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    private String getFomattedPose() {
        var pose = this.getState().Pose;
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        estimatedPose = m_odometry.getEstimatedPosition();
        
        field.setRobotPose(m_odometry.getEstimatedPosition());
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putData("Field", field);

        if (DebugConstants.debugMode) {
            SmartDashboard.putString("ROBOTPOSE", estimatedPose.toString());
            SmartDashboard.putNumber("ROBOTX", estimatedPose.getX());
            SmartDashboard.putNumber("ROBOTY", estimatedPose.getY());
        }        

        var dashboardPose = this.getState().Pose;
        if (originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = VisionHelpers.flipAlliance(dashboardPose);
        }
        field.setRobotPose(dashboardPose);
        if (DebugConstants.debugMode) SmartDashboard.putData(field);
    }

    public boolean isInRange() {
        return isInRange;
    }

    public void setInRange(boolean isInRange) {
        this.isInRange = isInRange;
    }

    @Override
    public void simulationPeriodic() {
        Subsystem.super.simulationPeriodic();
        field.setRobotPose(estimatedPose);
    }
}
