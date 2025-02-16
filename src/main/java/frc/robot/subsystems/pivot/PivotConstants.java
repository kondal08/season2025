package frc.robot.subsystems.pivot;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class PivotConstants {
  public static final int PIVOT_ID = 21;
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final DoubleSupplier kP = new LoggedTunableNumber("Pivot/kP", 0.02);
  public static final DoubleSupplier kI = new LoggedTunableNumber("Pivot/kI", 0);
  public static final DoubleSupplier kD = new LoggedTunableNumber("Pivot/kD", 0);
  public static final double ABSOLUTE_ENCODER_OFFSET = 0.38;
  public static final double RESTING_ANGLE = 0.0;
}
