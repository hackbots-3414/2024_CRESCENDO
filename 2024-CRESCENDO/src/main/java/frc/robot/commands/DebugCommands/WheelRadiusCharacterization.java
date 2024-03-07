package frc.robot.commands.DebugCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WheelRadiusCharacterization extends Command {
  CommandSwerveDrivetrain drivetrain;
  FieldCentric driveRequest = new SwerveRequest.FieldCentric();
   double driveBaseRadius = 0;
  double gyroStartingPosition;
  double gyroEndingPosition;

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

  }

  @Override
  public void initialize() {
    for (var moduleLocation : drivetrain.moduleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }
    SmartDashboard.putNumber("drivebase radius", driveBaseRadius);
    SmartDashboard.putNumber("Starting Gyro Position", Units.degreesToRadians(drivetrain.getPigeon2().getAngle()));

    gyroStartingPosition = Units.degreesToRadians(drivetrain.getPigeon2().getAngle());
  }

  public double getGyroPositionRadians() {
    return Units.degreesToRadians(drivetrain.getPigeon2().getAngle());
  }

  public double getWheelRotationRadAvg() {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("SWERVE MODULE MOVEMENT RADS" + i,
          (drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble() / 6.122448979591837) * Math.PI
              * 2.0);
              sum += (drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble() / 6.122448979591837) * Math.PI * 2.0;
    }
    return sum / 4;
  }

  @Override
  public void execute() {
    drivetrain.applyRequest(() -> driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(1.0));
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Ending Gyro Position", Units.degreesToRadians(drivetrain.getPigeon2().getAngle()));
    gyroEndingPosition = Units.degreesToRadians(drivetrain.getPigeon2().getAngle());
    double gyroPositionChange = gyroEndingPosition - gyroStartingPosition;
    double wheelRadius = (gyroPositionChange * driveBaseRadius) / getWheelRotationRadAvg();
    SmartDashboard.putNumber("Wheel Radius", wheelRadius);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
