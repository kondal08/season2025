// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.loadField;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.AllianceFlipUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
  public static final RobotMode MODE = RobotMode.REAL;
  public static final RobotType ROBOT = RobotType.COMPBOT;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static boolean TUNING_MODE = true;

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY,
  }

  public static enum RobotType {
    COMPBOT,
    DEVBOT,
    SIMBOT
  }

  /**
   * Checks whether the correct robot is selected when deploying (the main method is only ever
   * called on deploy).
   */
  public static void main(String... args) {
    if (ROBOT == RobotType.SIMBOT) {
      Alert alert =
          new Alert(
              "SIM robot loaded in REAL mode, gains likely breaking!", Alert.AlertType.kWarning);
      alert.set(true);
    }
  }

  public static final class FieldMap {
    public static enum Coordinates {
      SPEAKER(new Pose2d(0.05, 5.55, Rotation2d.fromDegrees(0))),
      AMP(new Pose2d(1.85, 7.6, Rotation2d.fromDegrees(270)));

      private final Pose2d pose;

      Coordinates(Pose2d pose) {
        this.pose = pose;
      }

      public Pose2d getPose() {
        return AllianceFlipUtil.apply(pose);
      }
    }

    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (11.25 / 12));
    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(54 + (3.25 / 12));

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        loadField(AprilTagFields.kDefaultField);
  }

  /** PID + FF gains, with overloaded constructors for disabling each term. */
  public record Gains(double kP, double kD, double kS, double kV, double kA, double kG) {
    public Gains(double kP, double kD, double kS, double kV, double kA) {
      this(kP, kD, kS, kV, kA, 0.0);
    }

    public Gains(double kP, double kD, double kS, double kV) {
      this(kP, kD, kS, kV, 0.0);
    }

    public Gains(double kP, double kD) {
      this(kP, kD, 0.0, 0.0);
    }
  }
}
