package frc.robot.subsystems.vision.apriltagvision;

import static frc.robot.subsystems.vision.apriltagvision.AprilTagVisionConstants.ROTATION_EULER_MULTIPLIERS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

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
        Math.exp(poseObservation.averageTagDistance() / poseObservation.tagCount()));
  }

  public static double eulerScale(PoseObservation poseObservation) {
    return ROTATION_EULER_MULTIPLIERS[poseObservation.tagCount()].getAsDouble()
        * poseObservation.ambiguity()
        / poseObservation.tagCount()
        * Math.exp(poseObservation.averageTagDistance() / poseObservation.tagCount());
  }
}
