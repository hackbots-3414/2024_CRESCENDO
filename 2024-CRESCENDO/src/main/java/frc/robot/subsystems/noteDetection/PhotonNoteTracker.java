// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteDetection;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonNoteTracker extends SubsystemBase {
  private PhotonCamera cam;
  /** Creates a new photonNoteTracker. */
  public PhotonNoteTracker() {
    cam = new PhotonCamera(Constants.VisionConstants.noteCameraName);
  }

  /**
   * Gets the note position as a Pose2d, but it's robot relative so that we can use it later!
   */
  private Optional<Pose2d> getNotePose() {
    PhotonPipelineResult latestResult = cam.getLatestResult();
    if (latestResult.hasTargets()) {
      PhotonTrackedTarget target = latestResult.getBestTarget();
      if (target.getPoseAmbiguity() > Constants.VisionConstants.maxNoteAmbiguity) {
        return Optional.empty();
      }
      Transform3d transform3d = target.getBestCameraToTarget();
      Pose2d goalPose = new Pose2d(new Translation2d(transform3d.getX(), transform3d.getY()), new Rotation2d(transform3d.getRotation().getZ()));
      return Optional.of(goalPose);
    } else {
      return Optional.empty();
    }
  }

  private Pose2d createBlueGoalPose2d(Pose2d robotPose, Pose2d notePose) {
    // rotation2d includes an invert angle because the camera is on the back
    Pose2d goalPose2d = new Pose2d(robotPose.getTranslation().plus(notePose.getTranslation()), robotPose.getRotation().minus(notePose.getRotation()));
    return goalPose2d;
  }

  private Pose2d createRedGoalPose2d(Pose2d robotPose, Pose2d notePose) {
    Pose2d bluePose = createBlueGoalPose2d(robotPose, notePose);
    Translation2d position = GeometryUtil.flipFieldPosition(bluePose.getTranslation());

    return new Pose2d(position, bluePose.getRotation());
  }

  public Pose2d getGoalPose(Supplier<Pose2d> robotPoseSup, boolean isRedSide) {
    Pose2d robotPose = robotPoseSup.get();
    Optional<Pose2d> notePoseMaybe = getNotePose();
    if (notePoseMaybe.isEmpty()) {
      return new Pose2d();
    }
    Pose2d notePose = notePoseMaybe.get();
    if (isRedSide) {
      return createRedGoalPose2d(robotPose, notePose);
    } else {
      return createBlueGoalPose2d(robotPose, notePose);
    }
  }
}
