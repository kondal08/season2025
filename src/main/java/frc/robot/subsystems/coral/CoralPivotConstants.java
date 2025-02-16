package frc.robot.subsystems.coral;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class CoralPivotConstants {
  public static final int PIVOT_ID = 21;
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 0.48, REVERSE_LIMIT = 0.1;

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final DoubleSupplier kP = new LoggedTunableNumber("Pivot/kP", 0.02);
}
