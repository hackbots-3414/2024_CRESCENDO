package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Objects;

/**
 * The VisionHelpers class provides utility methods and record classes for vision-related
 * operations.
 */
public class VisionHelpers {

    /**
     * Represents a pose estimate with additional information.
     *
     * @param pose The pose (position and orientation) estimate.
     * @param timestampSeconds The timestamp in seconds when the pose estimate was recorded.
     * @param averageTagDistance The average distance to the detected tags.
     * @param tagIDs The IDs of the detected tags.
     * @param poseAmbiguity Pose Ambiguity.
     */
    public record PoseEstimate(
            /** The pose (position and orientation) estimate. */
            Pose3d pose,
            /** The timestamp in seconds when the pose estimate was recorded. */
            double timestampSeconds,
            /** The average distance to the detected tags. */
            double averageTagDistance,
            /** The IDs of the detected tags. */
            int[] tagIDs,
            /** Ambiguity of Vision pose */
            double poseAmbiguity) {

        /**
         * Checks if this pose estimate is equal to another object.
         *
         * @param obj The object to compare.
         * @return True if the objects are equal, false otherwise.
         */
        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (obj == null || getClass() != obj.getClass()) {
                return false;
            }
            PoseEstimate other = (PoseEstimate) obj;
            return Arrays.equals(tagIDs, other.tagIDs)
                    && Objects.equals(pose, other.pose)
                    && Double.compare(timestampSeconds, other.timestampSeconds) == 0
                    && Double.compare(averageTagDistance, other.averageTagDistance) == 0
                    && Double.compare(poseAmbiguity, other.poseAmbiguity) == 0;
        }

        /**
         * Computes the hash code of this pose estimate.
         *
         * @return The hash code value.
         */
        @Override
        public int hashCode() {
            return Objects.hash(
                    Arrays.hashCode(getPose3dToArray(pose)),
                    timestampSeconds,
                    averageTagDistance,
                    Arrays.hashCode(tagIDs),
                    poseAmbiguity);
        }

        /**
         * Returns a string representation of this pose estimate.
         *
         * @return The string representation.
         */
        @Override
        public String toString() {
            return "PoseEstimate{"
                    + "pose="
                    + pose.toString()
                    + ", timestampSeconds="
                    + Double.toString(timestampSeconds)
                    + ", averageTagDistance="
                    + Double.toString(averageTagDistance)
                    + ", tagIDs="
                    + Arrays.toString(tagIDs)
                    + ", poseAmbiguity="
                    + Double.toString(poseAmbiguity)
                    + '}';
        }
    }

    /**
     * Converts a Pose3d object to an array of doubles.
     *
     * @param pose The Pose3d object to convert.
     * @return The array of doubles representing the pose.
     */
    public static double[] getPose3dToArray(Pose3d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = pose.getTranslation().getZ();
        result[3] = Units.radiansToDegrees(pose.getRotation().getX());
        result[4] = Units.radiansToDegrees(pose.getRotation().getY());
        result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
        return result;
    }
    /**
     * Converts a array of doubles  object to Pose3d.
     *
     * @param doubls[] The array of doubles representing the pose..
     * @return Pose3d from the array
     */
    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(
                        Units.degreesToRadians(inData[3]),
                        Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    /**
     * Represents a timestamped vision update with pose and standard deviations.
     *
     * @param timestamp The timestamp of the vision update.
     * @param pose The pose estimate.
     * @param stdDevs The standard deviations matrix.
     */
    public record TimestampedVisionUpdate(
            /** The timestamp of the vision update. */
            double timestamp,
            /** The pose estimate. */
            Pose2d pose,
            /** The standard deviations matrix. */
            Matrix<N3, N1> stdDevs) {

        /**
         * Returns a string representation of this vision update.
         *
         * @return The string representation.
         */
        @Override
        public String toString() {
            return "VisionUpdate{"
                    + "timestamp="
                    + Double.toString(timestamp)
                    + ", pose="
                    + pose.toString()
                    + ", stdDevs="
                    + stdDevs.toString()
                    + '}';
        }
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates
     * for each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    public static Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth), new Rotation2d(Math.PI)));
    }
}