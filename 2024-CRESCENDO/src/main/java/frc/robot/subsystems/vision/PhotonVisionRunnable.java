package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.util.FieldConstants;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonVisionRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
            new AtomicReference<EstimatedRobotPose>();

    public PhotonVisionRunnable(String photonCameraName, Transform3d robotToCamera) {
        this.photonCamera = new PhotonCamera(photonCameraName);

        PhotonPoseEstimator photonPoseEstimator = null;
        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // PV estimates will always be blue, they'll get flipped by robot thread
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if (photonCamera != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                    layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        // Get AprilTag data
        if (photonPoseEstimator != null && photonCamera != null/* && !RobotState.isAutonomous*/) {
            var photonResults = photonCamera.getLatestResult();
            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1
                            || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    // Make sure the measurement is on the field
                    if (estimatedPose.getX() > 0.0
                            && estimatedPose.getX() <= FieldConstants.fieldLength
                            && estimatedPose.getY() > 0.0
                            && estimatedPose.getY() <= FieldConstants.fieldWidth) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is
     * a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }
}