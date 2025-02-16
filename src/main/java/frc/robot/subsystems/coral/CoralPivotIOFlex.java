package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralPivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.coral.CoralPivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arm.GenericArmSystemIOSparkFlex;

public class CoralPivotIOFlex extends GenericArmSystemIOSparkFlex implements CoralPivotIO {

  public CoralPivotIOFlex() {
    super(
        new int[] {CoralPivotConstants.PIVOT_ID},
        new boolean[] {CoralPivotConstants.INVERTED},
        40,
        true,
        FORWARD_LIMIT,
        REVERSE_LIMIT,
        CoralPivotConstants.kP);
  }
}
