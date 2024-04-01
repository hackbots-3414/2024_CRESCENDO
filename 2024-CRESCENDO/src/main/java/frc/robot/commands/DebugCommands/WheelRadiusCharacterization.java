package frc.robot.commands.DebugCommands;

import java.math.BigDecimal;
import java.math.RoundingMode;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WheelRadiusCharacterization extends Command {
  private CommandSwerveDrivetrain drivetrain;

  private SwerveRequest request = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withVelocityX(1)
    .withVelocityY(0)
    .withRotationalRate(0);
  
  private double[] initialWheelRotation = new double[4];
  private double[] finalWheelRotation = new double[4];
  
  private double[] drivebaseRadii = new double[4];

  private double initialGyroDegrees;
  private double finalGyroDegrees;
  private BigDecimal drivebaseRotations;

  private BigDecimal fullRotationDegrees = new BigDecimal(360);

  private int precision = 12; // higher is more precise

  private Logger logger = LoggerFactory.getLogger(WheelRadiusCharacterization.class);
  
  public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
      for (int i = 0;i < 4;i ++) {
        TalonFX driveMotor = drivetrain.getModule(i).getDriveMotor();

        initialWheelRotation[i] = driveMotor.getPosition().getValueAsDouble();

        drivebaseRadii[i] = drivetrain.moduleLocations()[i].getNorm();
      }

      initialGyroDegrees = drivetrain.getPigeon2().getAngle();
      logger.debug("initialized");
  }

  @Override
  public void execute() {
    drivetrain.setControl(request);
    logger.debug("applied request");
    
    drivebaseRotations = new BigDecimal(drivetrain.getPigeon2().getAngle()).divide(fullRotationDegrees);

    // update each wheel's position
    for (int i = 0;i < 4;i ++) {
      finalWheelRotation[i] = drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble();
      SmartDashboard.putNumber("mod " + i + " rotation", finalWheelRotation[i] - initialWheelRotation[i]);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebaseRotations = getDriveBaseRotations();

    BigDecimal sum = new BigDecimal(0);

    for (int i = 0;i < 4;i ++) {
      sum = sum.add(calculateWheelRadius(i));
    }

    BigDecimal averageWheelRadius = sum.divide(new BigDecimal(4));

    BigDecimal averageWheelDiameter = averageWheelRadius.multiply(new BigDecimal(2));

    SmartDashboard.putString("Average Wheel Diameter", averageWheelDiameter.toPlainString());
    logger.info("Average wheel diameter: " + averageWheelDiameter.toPlainString());
  }

  private BigDecimal getDriveBaseRotations() {
    finalGyroDegrees = drivetrain.getPigeon2().getAngle();

    BigDecimal initialGyroRotations = new BigDecimal(initialGyroDegrees).divide(fullRotationDegrees, precision, RoundingMode.HALF_UP);
    BigDecimal finalGyroRotations = new BigDecimal(finalGyroDegrees).divide(fullRotationDegrees, precision, RoundingMode.HALF_UP);

    BigDecimal drivebaseRotations = finalGyroRotations.subtract(initialGyroRotations);

    logger.info("drive base rotations: " + drivebaseRotations.toPlainString());

    return drivebaseRotations;
  }

  private BigDecimal calculateWheelRadius(int index) {
    BigDecimal drivebaseRadius = new BigDecimal(drivebaseRadii[index]);

    BigDecimal wheelRotations = new BigDecimal(finalWheelRotation[index]).subtract(new BigDecimal(initialWheelRotation[index])).abs();

    BigDecimal wheelRadius = drivebaseRadius.multiply(drivebaseRotations).divide(wheelRotations, precision, RoundingMode.HALF_UP);

    logger.info("wheel radius for module " + index + ": " + wheelRadius.toPlainString());

    return wheelRadius;

  }
}
