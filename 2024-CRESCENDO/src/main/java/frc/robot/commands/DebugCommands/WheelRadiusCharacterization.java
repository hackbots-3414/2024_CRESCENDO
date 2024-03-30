package frc.robot.commands.DebugCommands;

import java.math.BigDecimal;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WheelRadiusCharacterization extends Command {
  CommandSwerveDrivetrain drivetrain;
  FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double driveBaseRadius = 37.268; //centemeters
  double gyroStartingPosition;
  double gyroEndingPosition;
  Logger log = LoggerFactory.getLogger(WheelRadiusCharacterization.class);

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    log.debug("initialized");
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

  public BigDecimal getWheelRotationRadAvg() {
    BigDecimal sum = new BigDecimal(0);
    for (int i = 0; i < 4; i++) {
      BigDecimal drivetrainRotationsbig = new BigDecimal(drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble());
      BigDecimal rotationsRad = drivetrainRotationsbig.divide(TunerConstants.kDriveGearRatioBig).multiply(new BigDecimal(Math.PI).multiply(new BigDecimal(2)));
      SmartDashboard.putNumber("SWERVE MODULE MOVEMENT RADS" + i, rotationsRad.doubleValue());
      sum = sum.add(rotationsRad);
    }
    return sum.divide(new BigDecimal(4));
  }

  @Override
  public void execute() {
    log.trace("executed");
    //drivetrain.applyRequest(() -> driveRequest.withVelocityX(5).withVelocityY(5).withRotationalRate(5));
    drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(1));
    
  }

  @Override
  public void end(boolean interrupted) {
    log.debug("command ended");
    SmartDashboard.putNumber("Ending Gyro Position", Units.degreesToRadians(drivetrain.getPigeon2().getAngle()));
    gyroEndingPosition = Units.degreesToRadians(drivetrain.getPigeon2().getAngle());
    BigDecimal gyroPositionChange = new BigDecimal(gyroEndingPosition).subtract(new BigDecimal(gyroStartingPosition));
    BigDecimal driveBaseRadiusBig = new BigDecimal(driveBaseRadius);

    BigDecimal wheelRadius = gyroPositionChange.multiply(driveBaseRadiusBig).divide(getWheelRotationRadAvg()).divide(new BigDecimal(2));

    SmartDashboard.putString("Wheel Radius (cm)", wheelRadius.toPlainString());
    log.info("I got: " + wheelRadius.toPlainString());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
