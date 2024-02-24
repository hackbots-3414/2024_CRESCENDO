package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
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
    

}
