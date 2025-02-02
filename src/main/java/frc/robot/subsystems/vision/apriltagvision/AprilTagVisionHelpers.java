package frc.robot.subsystems.vision.apriltagvision;

import static frc.robot.subsystems.vision.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

/**
 * This class provides utility methods and record classes for vision-related operations,
 * specifically for pose estimation using April Tags.
 */
public class AprilTagVisionHelpers {
  /**
   * Cool idea by the vision template & the PoseEstimator docs – dynamically update the vision
   * standard deviations based on the distance. In this case, we're scaling the deviations
   * quadratically with distance, so faraway tags are trusted <i>way</i> less than close ones.
   */
  public static Matrix<N3, N1> generateDynamicStdDevs(
      PoseObservation observation, int cameraIndex) {
    // Calculate standard deviations
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = TRANSLATION_EULER_MULTIPLIER.getAsDouble() * stdDevFactor;
    double angularStdDev = ROTATION_EULER_MULTIPLIER.getAsDouble() * stdDevFactor;

    if (cameraIndex < CAMERA_AMBIGUITY_FACTORS.length) {
      linearStdDev *= CAMERA_AMBIGUITY_FACTORS[cameraIndex].getAsDouble();
      angularStdDev *= CAMERA_AMBIGUITY_FACTORS[cameraIndex].getAsDouble();
    }

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
