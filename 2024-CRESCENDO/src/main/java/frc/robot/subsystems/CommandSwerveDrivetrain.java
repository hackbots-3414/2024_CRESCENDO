package frc.robot.subsystems;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
     private static final Logger LOGGER = LoggerFactory.getLogger(CommandSwerveDrivetrain.class);
    private static final double kSimLoopPeriod = 0.002; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field;

    private Pose2d estimatedPose;
    private boolean isInRange;
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.estimatedPose = new Pose2d();
        if (Robot.isSimulation()) {
            field = new Field2d();
            SmartDashboard.putData("Field", field);
            startSimThread();
        }
        setCurrentLimit(SwerveConstants.driveSupplyCurrentLimit);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.estimatedPose = new Pose2d();
        if (Robot.isSimulation()) {
            field = new Field2d();
            SmartDashboard.putData("Field", field);
            startSimThread();
        }
        setCurrentLimit(SwerveConstants.driveSupplyCurrentLimit);
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

    @Override
    public void periodic() {
        estimatedPose = m_odometry.getEstimatedPosition();
        field.setRobotPose(estimatedPose);
        SmartDashboard.putData(field);
        
        SmartDashboard.putString("ROBOTPOSE", estimatedPose.toString());
        SmartDashboard.putNumber("ROBOTX", estimatedPose.getX());
        SmartDashboard.putNumber("ROBOTY", estimatedPose.getY());
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

     public Command makeDriveToPoseCommand(Pose2d goalPose, boolean isReversed) {
        GoalEndState goal = DriverStation.getAlliance().get() == Alliance.Blue
                ? new GoalEndState(0.0, goalPose.getRotation())
                : new GoalEndState(0, new Rotation2d(Math.PI - goalPose.getRotation().getRadians()));
        PathPlannerPath path = new PathPlannerPath(PathPlannerPath.bezierFromPoses(estimatedPose, goalPose),
                new PathConstraints(SwerveConstants.maxDriveVelocity, SwerveConstants.maxDriveAcceleration,
                        SwerveConstants.maxAngleVelocity, SwerveConstants.maxAngleAcceleration),
                goal,
                isReversed);
        if (Robot.isSimulation()) {
          //  field.getRobotObject().setPoses(path.getPathPoses());
        }
        LOGGER.debug("makeDriveToPoseCommand: Calculated Poses: {}", path.getPathPoses());

        return AutoBuilder.followPath(path);
    }

    

}
