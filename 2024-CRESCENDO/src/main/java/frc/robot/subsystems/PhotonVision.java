// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase implements AutoCloseable {
  private PhotonCamera cameraLeft;
  private PhotonCamera cameraRight;
  private AprilTagFieldLayout field;
  private PhotonPoseEstimator estimator;

  /**
   * Creates a new PhotonVision.
   * 
   **/
  public PhotonVision() {
    cameraLeft = new PhotonCamera(Constants.VisionConstants.leftCameraName);
    cameraRight = new PhotonCamera(Constants.VisionConstants.rightCameraName);

    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Error while opening file for april tag locations. Sincerely, PhotonVision.java");
      System.out.println(AprilTagFields.k2024Crescendo.m_resourceFile);
    }

    estimator = new PhotonPoseEstimator(field, Constants.VisionConstants.mainStrategy, Constants.VisionConstants.leftTransform);
    estimator.setMultiTagFallbackStrategy(Constants.VisionConstants.fallbackStrategy);
  }

  private void filterPipelineResultByTargetID(PhotonPipelineResult original) {
    if (Constants.VisionConstants.validTagIds == null) return;
    original.targets.removeIf(target -> (!Constants.VisionConstants.validTagIds.contains(target.getFiducialId())));
  }

  private void filterPipelineResultByAmbiguity(PhotonPipelineResult original) {
    original.targets.removeIf(target -> (target.getPoseAmbiguity() > Constants.VisionConstants.maxAmbiguity));
  }

  private void filterPipelineResult(PhotonPipelineResult original) {
    filterPipelineResultByAmbiguity(original);
    filterPipelineResultByTargetID(original);
    // create more methods if we want, but keep them in their own separate methods for easier code management.
  }

  private PhotonPipelineResult getFilteredResult(PhotonCamera camera) {
    PhotonPipelineResult result = camera.getLatestResult();
    filterPipelineResult(result);
    return result;
  }


  public Optional<EstimatedRobotPose> getGlobalPoseFromLeft() {
    return estimator.update(getFilteredResult(cameraLeft));
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFromRight() {
    return estimator.update(getFilteredResult(cameraRight));
  }

  @Override
  public void close() throws Exception {
    cameraLeft.close();
    cameraRight.close();
  }
}
