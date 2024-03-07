package frc.robot.commands.DebugCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WheelRadiusCharacterization extends Command {
  CommandSwerveDrivetrain drivetrain;

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

  }

  @Override
  public void initialize() {
    double driveBaseRadius = 0;
    for (var moduleLocation : drivetrain.moduleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }
    SmartDashboard.putNumber("drivebase radius", driveBaseRadius);
    SmartDashboard.putNumber("Starting Gyro Position", Units.degreesToRadians(drivetrain.getPigeon2().getAngle()));
  }
  
  public double getGyroPositionRadians() {
    return Units.degreesToRadians(drivetrain.getPigeon2().getAngle());
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
        SmartDashboard.putNumber("Ending Gyro Position", Units.degreesToRadians(drivetrain.getPigeon2().getAngle()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
