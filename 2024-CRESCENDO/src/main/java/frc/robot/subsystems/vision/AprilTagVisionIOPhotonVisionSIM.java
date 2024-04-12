// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonVisionSIM implements AprilTagVisionIO {
    private final PhotonCamera rightCamera;
    private final PhotonCamera leftCamera;
    private final PhotonCamera backCamera;

    private final PhotonPoseEstimator photonEstimatorRight;
    private final PhotonPoseEstimator photonEstimatorLeft;
    private final PhotonPoseEstimator photonEstimatorBack;

    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSimRight;
    private PhotonCameraSim cameraSimLeft;
    private PhotonCameraSim cameraSimBack;

    private double lastEstTimestamp = 0;
    private final Supplier<Pose2d> poseSupplier;

    /**
     * Constructs a new AprilTagVisionIOPhotonVisionSIM instance.
     *
     * @param identifier    The identifier of the PhotonCamera.
     * @param robotToCamera The transform from the robot's coordinate system to the
     *                      camera's
     *                      coordinate system.
     * @param poseSupplier  The supplier of the robot's pose.
     */
    public AprilTagVisionIOPhotonVisionSIM(Supplier<Pose2d> poseSupplier) {
        this.rightCamera = new PhotonCamera(Constants.VisionConstants.rightCameraName);
        this.leftCamera = new PhotonCamera(Constants.VisionConstants.leftCameraName);
        this.backCamera = new PhotonCamera(Constants.VisionConstants.backCameraName);

        photonEstimatorRight = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                rightCamera,
                Constants.VisionConstants.rightTransform);
        photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorLeft = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                leftCamera,
                Constants.VisionConstants.leftTransform);
        photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorBack = new PhotonPoseEstimator(
                FieldConstants.aprilTags, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                backCamera,
                Constants.VisionConstants.backTransform);
        photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(FieldConstants.aprilTags);
        // Create simulated camera properties. These can be set to mimic your actual
        // camera.

        cameraSimRight = new PhotonCameraSim(rightCamera);
        cameraSimLeft = new PhotonCameraSim(leftCamera);
        cameraSimBack = new PhotonCameraSim(backCamera);

        SimCameraProperties cameraProps = getCameraProp();

        cameraSimRight = new PhotonCameraSim(rightCamera, cameraProps);
        cameraSimLeft = new PhotonCameraSim(leftCamera, cameraProps);
        cameraSimBack = new PhotonCameraSim(backCamera, cameraProps);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSimRight, Constants.VisionConstants.rightTransform);
        visionSim.addCamera(cameraSimLeft, Constants.VisionConstants.leftTransform);
        visionSim.addCamera(cameraSimBack, Constants.VisionConstants.backTransform);

        cameraSimRight.enableDrawWireframe(true);
        cameraSimLeft.enableDrawWireframe(true);
        cameraSimBack.enableDrawWireframe(true);

        this.poseSupplier = poseSupplier;
    }

    public SimCameraProperties getCameraProp() {

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        return cameraProp;
    }

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {

        updatePoseEstimates(inputs, cameraSimRight, photonEstimatorRight);
        updatePoseEstimates(inputs, cameraSimLeft, photonEstimatorLeft);
        updatePoseEstimates(inputs, cameraSimBack, photonEstimatorBack);
    }

    /** Updates the PhotonPoseEstimator and returns the estimated global pose. */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        visionEst.ifPresentOrElse(
                est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()), () -> {
                    if (newResult)
                        getSimDebugField().getObject("VisionEstimation").setPoses();
                });
        if (newResult)
            lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public void updatePoseEstimates(
            AprilTagVisionIOInputs inputs, PhotonCameraSim cameraSim, PhotonPoseEstimator photonEstimator) {

        visionSim.update(poseSupplier.get());
        PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
        ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
        double timestamp = results.getTimestampSeconds();
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        if (!results.targets.isEmpty() && allianceOptional.isPresent()) {
            double latencyMS = results.getLatencyMillis();
            Pose3d poseEstimation;
            Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(cameraSim.getCamera(), photonEstimator);
            if (estimatedPose.isEmpty()) {
                return;
            }
            poseEstimation = estimatedPose.get().estimatedPose;
            double averageTagDistance = 0.0;
            timestamp -= (latencyMS / 1e3);
            int[] tagIDs = new int[results.targets.size()];
            for (int i = 0; i < results.targets.size(); i++) {
                tagIDs[i] = results.targets.get(i).getFiducialId();
                var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
                if (tagPose.isEmpty()) {
                    continue;
                }
                averageTagDistance += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(poseEstimation.getTranslation().toTranslation2d());
            }
            averageTagDistance /= tagIDs.length;
            poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs, 0));
            inputs.poseEstimates = poseEstimates;
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!RobotBase.isSimulation())
            return null;
        return visionSim.getDebugField();
    }
}