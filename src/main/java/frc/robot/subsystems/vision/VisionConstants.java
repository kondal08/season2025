package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
  /**
   * An enum containing every type of camera we might have on our robot, and thresholds of distances
   * to tags at which we think they will provide meaningful data on a target.
   */
  public static enum CameraType {
    OV2311(5.0),
    OV9281(5.0),
    LIMELIGHT(2.0),
    LIMELIGHT_3G(3.0),
    TELEPHOTO_OV2311(5.0),
    TELEPHOTO_OV9281(5.0),
    TELEPHOTO_LIMELIGHT(5.0),
    TELEPHOTO_LIMELIGHT_3G(5.0),
    UNKNOWN(0.0);

    public final double noisyDistance;

    CameraType(double noisyDistance) {
      this.noisyDistance = noisyDistance;
    }
  }

  /**
   * A data class representing all the information we need on a camera to get us from it being
   * connected and streaming, to a filtered pose estimate we can fuse with other cameras' estimates
   * and the drivetrain's odometry.
   *
   * @param cameraName the NetworkTables name for this camera. This is what will help us identify
   *     each camera on AdvantageScope and Elastic, to help with debugging.
   * @param robotToCamera the transformation representing the camera's position relative to the
   *     center of the robot. Translations are in meters.
   * @param cameraType the type of hardware camera we are using. See {@link CameraType}.
   */
  public record CameraConstants(
      String cameraName, Transform3d robotToCamera, CameraType cameraType) {}
}
