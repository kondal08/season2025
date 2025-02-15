package frc.robot.subsystems.pivot;

import frc.robot.generic.arm.GenericArmSystemIOSparkFlex;

public class PivotIOFlex extends GenericArmSystemIOSparkFlex implements PivotIO {

  public PivotIOFlex() {
    super(
        new int[] {PivotConstants.PIVOT_ID},
        new boolean[] {PivotConstants.INVERTED},
        40,
        0.0,
        true,
        1.0,
        PivotConstants.kP,
        PivotConstants.kI,
        PivotConstants.kD);
  }
}
