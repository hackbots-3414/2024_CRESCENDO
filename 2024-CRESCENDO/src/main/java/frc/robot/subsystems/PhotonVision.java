// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class PhotonVision extends SubsystemBase {
  private PhotonCamera cameraLeft;
  private PhotonCamera cameraRight;

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    cameraLeft = new PhotonCamera("FIXME"); //FIXME Replace this with the actual camera name (Preferably in Constants.java)
    cameraRight = new PhotonCamera("FIXME AGAIN");
  }

  private PhotonPipelineResult getLeftResult() {
    return cameraLeft.getLatestResult();
  }
  private PhotonPipelineResult getRightResult() {
    return cameraRight.getLatestResult();
  }

  private List<Pose3d> getAllTargetsLocally() {
    List<Pose3d> targets = new ArrayList<>();

    PhotonPipelineResult leftResult = getLeftResult();
    PhotonPipelineResult rightResult = getRightResult();

    if (leftResult.hasTargets()) {
      for (PhotonTrackedTarget target : leftResult.getTargets()) {
        Transform3d best = target.getBestCameraToTarget();
        Pose3d pose = new Pose3d(best.getTranslation(), best.getRotation());
        targets.add(pose.transformBy(Constants.VisionConstants.leftTransform));

        // The target position is now relative to robot center facing forwards
        
      }
    }

    if (rightResult.hasTargets()) {
      for (PhotonTrackedTarget target : rightResult.getTargets()) {
        Transform3d best = target.getBestCameraToTarget();
        Pose3d pose = new Pose3d(best.getTranslation(), best.getRotation());
        targets.add(pose.transformBy(Constants.VisionConstants.rightTransform));
      }
    }

    return targets; // All targets are relative to robot center. Orientation should also be corrected so that all of the poses we have should be as if they were taken from the center of the robot facing forwards.
  }
}
