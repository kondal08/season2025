package frc.robot.subsystems.vision.apriltagvision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

// TODO tune all of these!!
public final class AprilTagVisionConstants {
  public static final boolean LEFT_CAM_ENABLED = false;
  public static final VisionConstants.CameraConstants LEFT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "lefttagcam",
          new Transform3d(
              new Translation3d(0.3, -0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-25), degreesToRadians(0))),
          VisionConstants.CameraType.TELEPHOTO_OV9281);

  public static final boolean RIGHT_CAM_ENABLED = false;
  public static final VisionConstants.CameraConstants RIGHT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "righttagcam",
          new Transform3d(
              new Translation3d(0.3, 0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-25), degreesToRadians(0))),
          VisionConstants.CameraType.TELEPHOTO_OV9281);

  public static final DoubleSupplier TRANSLATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Translation", 0.02);
  public static final DoubleSupplier ROTATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Rotation", 0.06);

  public static final double MAX_AMBIGUITY_CUTOFF = 0.3;
  public static final double MAX_Z_ERROR = 0.75;
  public static final double MAX_DISTANCE_CUTOFF = 5;
  /**
   * Rough values for how much we trust this camera to produce reliable data on our target relative
   * to the other cameras. A lower value means we trust this camera more - for instance, if we're
   * more confident in its calibration than the other cameras. If any cameras are not explicitly
   * listed here, they will have a deafult ambiguity factor of 1.0.
   */
  public static final DoubleSupplier[] CAMERA_AMBIGUITY_FACTORS =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber(
            "AprilTagVision/CameraAmbiguityFactors/" + LEFT_CAM_CONSTANTS.cameraName(), 1),
        new LoggedTunableNumber(
            "AprilTagVision/CameraAmbiguityFactors/" + RIGHT_CAM_CONSTANTS.cameraName(), 1)
      };
}
