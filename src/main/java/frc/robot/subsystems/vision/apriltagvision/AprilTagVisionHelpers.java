package frc.robot.subsystems.vision.apriltagvision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class provides utility methods and record classes for vision-related operations,
 * specifically for pose estimation using April Tags.
 */
public class AprilTagVisionHelpers {
  /**
   * Cool idea by 8033, 5026, and the PoseEstimator docs – dynamically update the vision stddevs
   * based on the distance. In this case, we're scaling the deviations exponentially with distance,
   * so faraway tags are trusted <i>way</i> less than close ones.
   */
  public static Matrix<N3, N1> calculateDeviations(PoseObservation poseObservation) {
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        Math.exp(poseObservation.averageTagDistance / poseObservation.tagCount));
  }

  public static double eulerScale(double eulerCoefficient, PoseObservation poseObservation) {
    return eulerCoefficient
        * poseObservation.ambiguity
        / poseObservation.tagCount
        * Math.exp(poseObservation.averageTagDistance / poseObservation.tagCount);
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}
}
