package frc.robot.subsystems.vision.apriltagvision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants;

// TODO tune all of these!!
public final class AprilTagVisionConstants {

  public static final boolean LEFT_CAM_ENABLED = true;
  public static final VisionConstants.CameraConstants LEFT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "lefttagcam",
          new Transform3d(
              new Translation3d(0.306, -0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-28), degreesToRadians(-30))),
          VisionConstants.CameraType.TELEPHOTO_OV9281,
          1);

  public static final boolean RIGHT_CAM_ENABLED = true;
  public static final VisionConstants.CameraConstants RIGHT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "righttagcam",
          new Transform3d(
              new Translation3d(0.306, 0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-28), degreesToRadians(30))),
          VisionConstants.CameraType.TELEPHOTO_OV9281,
          1);

  //      public static final UnitDeviationParams MOVING_DEVIATION_PARAMS =
  //          new UnitDeviationParams(
  //              MOVING_DEVIATION_VELOCITY_MULTIPLIER, MOVING_DEVIATION_EULER_MULTIPLIER, 1);
  public static final double MOVING_DEVIATION_EULER_MULTIPLIER = 0.5;
  public static final double MOVING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
  public static final double TURNING_DEVIATION_EULER_MULTIPLIER = 0.5;
  public static final double TURNING_DEVIATION_VELOCITY_MULTIPLIER = 0.5;
  static final double MAX_AMBIGUITY_CUTOFF = 0.05;
}
