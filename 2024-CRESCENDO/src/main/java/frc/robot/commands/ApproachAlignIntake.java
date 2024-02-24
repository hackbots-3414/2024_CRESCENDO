// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.NoteFinder.Gamepiece;
import frc.robot.subsystems.Transport;
import com.pathplanner.lib.util.GeometryUtil;

public class ApproachAlignIntake extends Command {
  private static final Logger LOGGER = LoggerFactory.getLogger(ApproachAlignIntake.class);
  private NoteFinder noteFinder = null;
  private Intake intake = null;
  private Transport transport = null;
  private CommandSwerveDrivetrain drivetrain = null;
  private Gamepiece gamepiece = null;
  private Command nextCommand = null;

  /** Creates a new ApproachAlignIntake. */
  public ApproachAlignIntake(NoteFinder noteFinder, Intake intake, Transport transport,
      CommandSwerveDrivetrain drivetrain) {
    this.intake = intake;
    this.noteFinder = noteFinder;
    this.transport = transport;
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, transport, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private Pose2d createBlueGoalPose2d() {
    gamepiece = noteFinder.getGamepieces()[0];
    // rotation2d includes an invert angle because the camera is on the back
    Rotation2d rotation2d = Rotation2d
        .fromDegrees(drivetrain.getPose().getRotation().getDegrees() - gamepiece.getAngle());
    // Put safety checks on the distance
    // Made change on PathPlanner NavGrid
    Translation2d targetGoal = new Translation2d(-3, rotation2d);
    Pose2d goalPose2d = new Pose2d(drivetrain.getPose().getTranslation().plus(targetGoal), rotation2d);
    LOGGER.trace("currentPose: {}, goalPose2d: {}, gamepiece: {}", drivetrain.getPose(), goalPose2d, gamepiece);
    return goalPose2d;
  }

  private Pose2d createRedGoalPose2d() {
    Pose2d bluePose = createBlueGoalPose2d();
    Translation2d position = GeometryUtil.flipFieldPosition(bluePose.getTranslation());

    return new Pose2d(position, bluePose.getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LOGGER.debug("end");
    if (!noteFinder.isNoteDetected()) {
      return;
    }
    intake.activateIntake();
    transport.activateTransport();
    nextCommand = drivetrain.makeDriveToPoseCommand(DriverStation.getAlliance().get() == Alliance.Blue ? createBlueGoalPose2d() : createRedGoalPose2d(), true)
        .until(transport::isNoteDetected)
        .finallyDo((boolean end) -> {
          intake.stopMotor();
          transport.stopMotor();
        });
        nextCommand.schedule();
    LOGGER.trace("end exit");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}