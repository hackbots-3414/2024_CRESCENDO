// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera cameraLeft;
  private PhotonCamera cameraRight;
  private AprilTagFieldLayout field;
  private PoseStrategy strategy;
  private PhotonPoseEstimator leftEstimator;
  private PhotonPoseEstimator rightEstimator;

  /** Creates a new PhotonVision. 
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
    strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    leftEstimator = new PhotonPoseEstimator(field, strategy, cameraLeft, Constants.VisionConstants.leftTransform);
    rightEstimator = new PhotonPoseEstimator(field, strategy, cameraRight, Constants.VisionConstants.rightTransform);
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFromLeft() {
    return leftEstimator.update();
  }

  public Optional<EstimatedRobotPose> getGlobalPoseFromRight() {
    return rightEstimator.update();
  }
}
